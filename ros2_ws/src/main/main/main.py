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

BAUDRATE = 115200             # ボーレート 115200
DEVICENAME = '/dev/ttyUSB0'  # 使用環境に合わせて調整してください
MAX_VELOCITY = 200            # 通常時の最高速度

# --- PS4 コントローラー（joy_node）のインデックス定義 ---
AXIS_LEFT_STICK_Y = 1
AXIS_L2 = 2
AXIS_R2 = 5
BUTTON_CIRCLE = 1             # 〇ボタン (エアシリンダー制御用)
BUTTON_OPTIONS = 9            # OPTIONSボタン (緊急停止用)

# --- 辞書型データ構造（設計書拡張） ---
robot_a = {
    'name': 'robot_a',
    'joy_topic': '/joy_a',
    'cylinder_topic': '/cylinder_state_a',
    'dxl_ids': [2],
    'invert': [1],
    'target_vel': [0],
    'slow_mode': False,
    'emergency_stop': False,
    'cylinder_state': False,  # False: OFF, True: ON
    'prev_button_circle': 0   # 前回〇ボタン状態
}

robot_b = {
    'name': 'robot_b',
    'joy_topic': '/joy_b',
    'cylinder_topic': '/cylinder_state_b',
    'dxl_ids': [5],
    'invert': [1],
    'target_vel': [0],
    'slow_mode': False,
    'emergency_stop': False,
    'cylinder_state': False,  # False: OFF, True: ON
    'prev_button_circle': 0   # 前回〇ボタン状態
}


# --- 1. ヘルパー関数 ---
def apply_deadzone(val, threshold=0.05):
    """スティックの不感帯処理"""
    if abs(val) < threshold:
        return 0.0
    return val


# --- 2. ロジック処理関数 ---
def update_robot_control(robot_dict, joy_msg, logger, cylinder_pub):
    """Dynamixelの目標速度計算およびエアシリンダーのトグル処理を行う"""
    
    # --- A. エアシリンダー制御 (〇ボタンの立ち上がりエッジ検出) ---
    if len(joy_msg.buttons) > BUTTON_CIRCLE:
        current_circle_state = joy_msg.buttons[BUTTON_CIRCLE]
        
        # 0 -> 1 になった瞬間 (押された瞬間)
        if current_circle_state == 1 and robot_dict['prev_button_circle'] == 0:
            robot_dict['cylinder_state'] = not robot_dict['cylinder_state']
            
            state_str = "ON" if robot_dict['cylinder_state'] else "OFF"
            logger.info(f"[{robot_dict['name']}] Circle button pressed! Cylinder: {state_str}")
            
            # シリンダー状態のパブリッシュ
            bool_msg = Bool()
            bool_msg.data = robot_dict['cylinder_state']
            cylinder_pub.publish(bool_msg)

        robot_dict['prev_button_circle'] = current_circle_state

    # --- B. 緊急停止チェック (OPTIONSボタン) ---
    if len(joy_msg.buttons) > BUTTON_OPTIONS:
        if joy_msg.buttons[BUTTON_OPTIONS] == 1:
            if not robot_dict['emergency_stop']:
                logger.error(f"[{robot_dict['name']}] EMERGENCY STOP TRIGGERED!")
                robot_dict['emergency_stop'] = True

    # 緊急停止中の場合は速度 0 で終了
    if robot_dict['emergency_stop']:
        robot_dict['target_vel'] = [0]
        return

    # --- C. 減速モード判定 (L2 / R2 トリガー) ---
    robot_dict['slow_mode'] = False
    if len(joy_msg.axes) > max(AXIS_L2, AXIS_R2):
        if joy_msg.axes[AXIS_L2] < 0.0 or joy_msg.axes[AXIS_R2] < 0.0:
            robot_dict['slow_mode'] = True

    # --- D. 速度計算 (Dynamixel) ---
    if len(joy_msg.axes) > AXIS_LEFT_STICK_Y:
        raw_val = joy_msg.axes[AXIS_LEFT_STICK_Y]
        val = apply_deadzone(raw_val)

        speed_limit = MAX_VELOCITY * 0.5 if robot_dict['slow_mode'] else MAX_VELOCITY
        target_vel = int(val * speed_limit * robot_dict['invert'][0])
        
        robot_dict['target_vel'] = [target_vel]


class DualRobotController(Node):
    def __init__(self, port_handler, packet_handler, group_sync_write):
        super().__init__('dual_robot_controller')
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.group_sync_write = group_sync_write

        # --- サブスクライバー設定 (/joy_a, /joy_b) ---
        self.sub_a = self.create_subscription(
            Joy, robot_a['joy_topic'], self.joy_a_callback, 10)
        self.sub_b = self.create_subscription(
            Joy, robot_b['joy_topic'], self.joy_b_callback, 10)

        # --- パブリッシャー設定 (シリンダー用) ---
        self.pub_cylinder_a = self.create_publisher(Bool, robot_a['cylinder_topic'], 10)
        self.pub_cylinder_b = self.create_publisher(Bool, robot_b['cylinder_topic'], 10)

        # --- 定期送信タイマー (20Hz / 0.05秒周期) ---
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Integrated Robot Controller (Dynamixel + Air Cylinder) started.")

    def joy_a_callback(self, msg):
        update_robot_control(robot_a, msg, self.get_logger(), self.pub_cylinder_a)

    def joy_b_callback(self, msg):
        update_robot_control(robot_b, msg, self.get_logger(), self.pub_cylinder_b)

    def timer_callback(self):
        """Dynamixel SDKの GroupSyncWrite を使用して速度命令を一括送信"""
        self.group_sync_write.clearParam()

        for robot in [robot_a, robot_b]:
            for dxl_id, vel in zip(robot['dxl_ids'], robot['target_vel']):
                # 負の速度値を2の補数表現（4バイト）に変換
                vel_bytes = [(vel >> (8 * i)) & 0xFF for i in range(4)]
                
                add_param_result = self.group_sync_write.addParam(dxl_id, vel_bytes)
                if not add_param_result:
                    self.get_logger().error(f"Failed to add param for Dynamixel ID {dxl_id}")

        # 一括送信
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != 0:
            self.get_logger().error(
                f"SyncWrite failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")


# --- 全体管理関数 ---
def main(args=None):
    rclpy.init(args=args)

    # シリアル通信・Dynamixelプロトコル初期化
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(2.0)
    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

    if not port_handler.openPort():
        print(f"Failed to open port: {DEVICENAME}")
        sys.exit(1)

    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate: {BAUDRATE}")
        sys.exit(1)

    # 各Dynamixelのモード設定とトルクON
    test_ids = robot_a['dxl_ids'] + robot_b['dxl_ids']
    for dxl_id in test_ids:
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    node = DualRobotController(port_handler, packet_handler, group_sync_write)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        # 終了処理：Dynamixel速度 0 送信 & トルクOFF
        group_sync_write.clearParam()
        stop_bytes = [0, 0, 0, 0]
        for dxl_id in test_ids:
            group_sync_write.addParam(dxl_id, stop_bytes)
        group_sync_write.txPacket()

        for dxl_id in test_ids:
            packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
