import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

# --- Dynamixel 定数定義 (Xシリーズ Velocity Control Mode) ---
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104

LEN_GOAL_VELOCITY = 4
VELOCITY_MODE = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'
MAX_VELOCITY = 200

# --- PS4 コントローラー インデックス定義 ---
AXIS_LEFT_STICK_X = 0   # 左スティック X軸: X軸移動
AXIS_LEFT_STICK_Y = 1   # 左スティック Y軸: Y1軸移動
AXIS_RIGHT_STICK_Y = 4  # 右スティック Y軸: Y2軸移動
AXIS_L2 = 2             # L2トリガー: 微修正モード
AXIS_R2 = 5             # R2トリガー: 微修正モード

BUTTON_CIRCLE = 1       # ○ボタン: エアシリンダー トグル制御
BUTTON_OPTIONS = 9      # OPTIONSボタン: 緊急停止

# --- 辞書型データ構造 (設計書に準拠) ---
robot_a = {
    'name': 'robot_a',
    'joy_topic': '/joy_a',
    'cylinder_topic': '/cylinder_state_a',
    'dxl_ids': [1, 2, 3],      # [X軸, Y1軸, Y2軸]
    'invert': [1, -1, 1],      # 回転方向反転フラグ
    'target_vel': [0, 0, 0],   # [X軸速度, Y1軸速度, Y2軸速度]
    'slow_mode': False,        # 微修正モード (L2/R2)
    'emergency_stop': False,   # 緊急停止状態
    'cylinder_state': False,   # エアシリンダー状態 (False: OFF, True: ON)
    'prev_button_circle': 0,   # ○ボタンの立ち上がり検出用
    'joy_connected': False     # Joyトピック受信確認フラグ
}

robot_b = {
    'name': 'robot_b',
    'joy_topic': '/joy_b',
    'cylinder_topic': '/cylinder_state_b',
    'dxl_ids': [4, 5, 6],      # [X軸, Y1軸, Y2軸]
    'invert': [1, -1, 1],      # 回転方向反転フラグ
    'target_vel': [0, 0, 0],   # [X軸速度, Y1軸速度, Y2軸速度]
    'slow_mode': False,
    'emergency_stop': False,
    'cylinder_state': False,
    'prev_button_circle': 0,
    'joy_connected': False
}


def apply_deadzone(val, threshold=0.05):
    """スティックの微小振動（ブレ）を無視する不感帯処理"""
    if abs(val) < threshold:
        return 0.0
    return val


def update_robot_velocity(robot_dict, joy_msg, logger, cylinder_pub):
    """受信したコントローラー入力から目標速度とシリンダー状態を更新"""
    # 初回受信時のログ出力
    if not robot_dict['joy_connected']:
        logger.info(f"SUCCESS: Signal received on [{robot_dict['joy_topic']}] for [{robot_dict['name']}]")
        robot_dict['joy_connected'] = True

    # --- 1. エアシリンダー トグル制御 (○ボタン) ---
    if len(joy_msg.buttons) > BUTTON_CIRCLE:
        current_circle = joy_msg.buttons[BUTTON_CIRCLE]
        # 立ち上がりエッジ検出 (前回0で今回1)
        if current_circle == 1 and robot_dict['prev_button_circle'] == 0:
            robot_dict['cylinder_state'] = not robot_dict['cylinder_state']
            state_str = "ON" if robot_dict['cylinder_state'] else "OFF"
            logger.info(f"[{robot_dict['name']}] Air Cylinder Toggled -> {state_str} (Topic: {robot_dict['cylinder_topic']})")
            
            # Boolメッセージの配信
            cmd_msg = Bool()
            cmd_msg.data = robot_dict['cylinder_state']
            cylinder_pub.publish(cmd_msg)

        robot_dict['prev_button_circle'] = current_circle

    # --- 2. 緊急停止チェック (OPTIONSボタン) ---
    if len(joy_msg.buttons) > BUTTON_OPTIONS:
        if joy_msg.buttons[BUTTON_OPTIONS] == 1:
            if not robot_dict['emergency_stop']:
                logger.fatal(f"[{robot_dict['name']}] EMERGENCY STOP TRIGGERED!")
                robot_dict['emergency_stop'] = True

    if robot_dict['emergency_stop']:
        robot_dict['target_vel'] = [0, 0, 0]
        return

    # --- 3. 微修正モード判定 (L2/R2トリガー) ---
    prev_slow = robot_dict['slow_mode']
    robot_dict['slow_mode'] = False
    if len(joy_msg.axes) > max(AXIS_L2, AXIS_R2):
        # PS4コントローラーのトリガーは未押しが 1.0、押し込みで -1.0
        if joy_msg.axes[AXIS_L2] < -0.5 or joy_msg.axes[AXIS_R2] < -0.5:
            robot_dict['slow_mode'] = True

    if prev_slow != robot_dict['slow_mode']:
        mode_str = "SLOW (50%)" if robot_dict['slow_mode'] else "NORMAL (100%)"
        logger.info(f"[{robot_dict['name']}] Speed Mode -> {mode_str}")

    # --- 4. 3軸(X, Y1, Y2)の目標速度計算 ---
    speed_limit = MAX_VELOCITY * 0.5 if robot_dict['slow_mode'] else MAX_VELOCITY

    # X軸 (左スティック X)
    val_x = apply_deadzone(joy_msg.axes[AXIS_LEFT_STICK_X]) if len(joy_msg.axes) > AXIS_LEFT_STICK_X else 0.0
    vel_x = int(val_x * speed_limit * robot_dict['invert'][0])

    # Y1軸 (左スティック Y)
    val_y1 = apply_deadzone(joy_msg.axes[AXIS_LEFT_STICK_Y]) if len(joy_msg.axes) > AXIS_LEFT_STICK_Y else 0.0
    vel_y1 = int(val_y1 * speed_limit * robot_dict['invert'][1])

    # Y2軸 (右スティック Y)
    val_y2 = apply_deadzone(joy_msg.axes[AXIS_RIGHT_STICK_Y]) if len(joy_msg.axes) > AXIS_RIGHT_STICK_Y else 0.0
    vel_y2 = int(val_y2 * speed_limit * robot_dict['invert'][2])

    robot_dict['target_vel'] = [vel_x, vel_y1, vel_y2]

    # スティック入力があった場合にデバッグログを出力
    if val_x != 0.0 or val_y1 != 0.0 or val_y2 != 0.0:
        logger.debug(f"[{robot_dict['name']}] Target Vel (X, Y1, Y2): {robot_dict['target_vel']}")


class DualRobotController(Node):
    def __init__(self, port_handler, packet_handler, group_sync_write):
        super().__init__('dual_robot_controller')
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.group_sync_write = group_sync_write

        # Subscribers
        self.sub_a = self.create_subscription(
            Joy, robot_a['joy_topic'], self.joy_a_callback, 10)
        self.sub_b = self.create_subscription(
            Joy, robot_b['joy_topic'], self.joy_b_callback, 10)

        # Publishers (エアシリンダー制御用)
        self.pub_cylinder_a = self.create_publisher(Bool, robot_a['cylinder_topic'], 10)
        self.pub_cylinder_b = self.create_publisher(Bool, robot_b['cylinder_topic'], 10)

        # 送信タイマー (20Hz / 0.05秒周期)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 接続監視タイマー (3秒周期)
        self.health_check_timer = self.create_timer(3.0, self.check_joy_health)

        self.get_logger().info("=== Dual Robot Controller Initialized (6 Dynamixels & 2 Air Cylinders) ===")

    def joy_a_callback(self, msg):
        update_robot_velocity(robot_a, msg, self.get_logger(), self.pub_cylinder_a)

    def joy_b_callback(self, msg):
        update_robot_velocity(robot_b, msg, self.get_logger(), self.pub_cylinder_b)

    def check_joy_health(self):
        """コントローラーからのトピック受信状況を監視"""
        for r in [robot_a, robot_b]:
            if not r['joy_connected']:
                self.get_logger().warn(f"Waiting for input on [{r['name']}] topic ({r['joy_topic']})...")

    def timer_callback(self):
        """Dynamixel SDK GroupSyncWrite を使用して 6軸一括送信"""
        self.group_sync_write.clearParam()

        for robot in [robot_a, robot_b]:
            for dxl_id, vel in zip(robot['dxl_ids'], robot['target_vel']):
                # 32ビット整数値を4バイトに分解
                vel_bytes = [(vel >> (8 * i)) & 0xFF for i in range(4)]
                add_param_result = self.group_sync_write.addParam(dxl_id, vel_bytes)
                if not add_param_result:
                    self.get_logger().error(f"Failed to add param for Dynamixel ID {dxl_id}")

        # 一括送信
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != 0:
            self.get_logger().error(
                f"GroupSyncWrite failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")


def main(args=None):
    rclpy.init(args=args)

    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(2.0)
    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

    # ポートのオープン
    if not port_handler.openPort():
        print(f"[FATAL ERROR] Failed to open serial port: {DEVICENAME}")
        sys.exit(1)

    # ボーレートの設定
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"[FATAL ERROR] Failed to set baudrate: {BAUDRATE}")
        sys.exit(1)

    all_dxl_ids = robot_a['dxl_ids'] + robot_b['dxl_ids']
    
    # 6台のDynamixelのモード設定およびトルクON
    for dxl_id in all_dxl_ids:
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    node = DualRobotController(port_handler, packet_handler, group_sync_write)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Dual Robot Controller Node...')
    finally:
        # 終了処理：全軸速度0一括送信 & トルクオフ
        group_sync_write.clearParam()
        stop_bytes = [0, 0, 0, 0]
        for dxl_id in all_dxl_ids:
            group_sync_write.addParam(dxl_id, stop_bytes)
        group_sync_write.txPacket()

        for dxl_id in all_dxl_ids:
            packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()