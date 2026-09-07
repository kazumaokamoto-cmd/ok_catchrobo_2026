import sys
from functools import partial
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
SLOW_MODE_RATIO = 0.5          # Slow Mode (微修正モード) 時の速度倍率 (50%)

# --- 制御パラメータ定数 ---
JOY_DEADZONE = 0.05             # スティックの微小振動を無視する不感帯しきい値
PS4_TRIGGER_THRESHOLD = -0.5    # PS4 L2/R2 トリガー押下判定しきい値
JOY_TIMEOUT = 0.5               # Joyコントローラー通信タイムアウト判定時間 (秒)
COMM_TIMER_PERIOD = 0.05        # Dynamixel速度指令送信周期 (20Hz = 0.05秒)
HEALTH_CHECK_PERIOD = 0.1       # Joy接続監視タイマー周期 (10Hz = 0.1秒)

# --- PS4 コントローラー インデックス定義 ---
AXIS_LEFT_STICK_X = 0   # 左スティック X軸: X軸移動
AXIS_LEFT_STICK_Y = 1   # 左スティック Y軸: Y1軸移動
AXIS_RIGHT_STICK_Y = 4  # 右スティック Y軸: Y2軸移動
AXIS_L2 = 2             # L2トリガー: 微修正モード
AXIS_R2 = 5             # R2トリガー: 微修正モード

BUTTON_CROSS = 0        # ×ボタン: 真空ポンプ トグル制御
BUTTON_CIRCLE = 1       # ○ボタン: エアシリンダー トグル制御
BUTTON_OPTIONS = 9      # OPTIONSボタン: 緊急停止

# --- 辞書型データ構造 (各ロボットの状態保持) ---
# 配列要素の順序統一: 0番目 = X軸, 1番目 = Y1軸, 2番目 = Y2軸
robot_a = {
    'name': 'robot_a',
    'joy_topic': '/joy_a',
    'dxl_ids': [1, 2, 3],      # [X軸, Y1軸, Y2軸]
    'invert': [1, -1, 1],      # [X軸, Y1軸, Y2軸] モーター回転方向反転フラグ (1: 正転, -1: 逆転)
    'target_vel': [0, 0, 0],   # [X軸速度, Y1軸速度, Y2軸速度]
    'slow_mode': False,        # 微修正モード状態 (L2/R2押下時 True)
    'emergency_stop': False,   # 緊急停止状態
    'cylinder_state': False,   # エアシリンダー状態 (False: OFF, True: ON)
    'pump_state': False,       # 真空ポンプ状態 (False: OFF, True: ON)
    'prev_button_circle': 0,   # ○ボタンの立ち上がり検出用前値
    'prev_button_cross': 0,    # ×ボタンの立ち上がり検出用前値
    'joy_connected': False,    # 初回Joy受信完了フラグ
    'last_joy_time': None,     # 最終Joy受信時刻 (秒)
    'is_timed_out': False      # Joy通信タイムアウト状態フラグ
}

robot_b = {
    'name': 'robot_b',
    'joy_topic': '/joy_b',
    'dxl_ids': [4, 5, 6],      # [X軸, Y1軸, Y2軸]
    'invert': [1, -1, 1],      # [X軸, Y1軸, Y2軸] モーター回転方向反転フラグ (1: 正転, -1: 逆転)
    'target_vel': [0, 0, 0],   # [X軸速度, Y1軸速度, Y2軸速度]
    'slow_mode': False,
    'emergency_stop': False,
    'cylinder_state': False,
    'pump_state': False,
    'prev_button_circle': 0,
    'prev_button_cross': 0,
    'joy_connected': False,
    'last_joy_time': None,
    'is_timed_out': False
}


# --- 小さな共通ヘルパー関数 ---

def apply_deadzone(val, threshold=JOY_DEADZONE):
    """スティックの微小振動（ブレ）を無視する不感帯処理"""
    if abs(val) < threshold:
        return 0.0
    return val


def get_button(joy_msg, index):
    """ボタン配列から指定インデックスの値を取得（範囲外アクセス防止）"""
    if len(joy_msg.buttons) > index:
        return joy_msg.buttons[index]
    return 0


def get_axis(joy_msg, index):
    """軸配列から指定インデックスの値を取得（範囲外アクセス防止）"""
    if len(joy_msg.axes) > index:
        return joy_msg.axes[index]
    return 0.0


# --- Joy入力処理の分割関数 ---

def update_robot_buttons(robot_dict, joy_msg, logger):
    """
    トグルボタン (シリンダー/ポンプ) および緊急停止ボタンの処理
    """
    # 1. エアシリンダー トグル制御 (○ボタン)
    current_circle = get_button(joy_msg, BUTTON_CIRCLE)
    # 立ち上がりエッジ検出 (前回0で今回1)
    if current_circle == 1 and robot_dict['prev_button_circle'] == 0:
        robot_dict['cylinder_state'] = not robot_dict['cylinder_state']
        state_str = "ON" if robot_dict['cylinder_state'] else "OFF"
        logger.info(f"[{robot_dict['name']}] Air Cylinder: {state_str}")

        # =========================================================================
        # TODO: ここに自作ライブラリのエアシリンダーON/OFF処理を入れてください
        # 例:
        # if robot_dict['cylinder_state']:
        #     your_lib.cylinder_on(robot_dict['name'])
        # else:
        #     your_lib.cylinder_off(robot_dict['name'])
        # =========================================================================

    robot_dict['prev_button_circle'] = current_circle

    # 2. 真空ポンプ トグル制御 (×ボタン)
    current_cross = get_button(joy_msg, BUTTON_CROSS)
    # 立ち上がりエッジ検出 (前回0で今回1)
    if current_cross == 1 and robot_dict['prev_button_cross'] == 0:
        robot_dict['pump_state'] = not robot_dict['pump_state']
        state_str = "ON" if robot_dict['pump_state'] else "OFF"
        logger.info(f"[{robot_dict['name']}] Vacuum Pump: {state_str}")

        # =========================================================================
        # TODO: ここに自作ライブラリの真空ポンプON/OFF処理を入れてください
        # 例:
        # if robot_dict['pump_state']:
        #     your_lib.pump_on(robot_dict['name'])
        # else:
        #     your_lib.pump_off(robot_dict['name'])
        # =========================================================================

    robot_dict['prev_button_cross'] = current_cross

    # 3. 緊急停止チェック (OPTIONSボタン)
    if get_button(joy_msg, BUTTON_OPTIONS) == 1:
        if not robot_dict['emergency_stop']:
            logger.fatal(f"[{robot_dict['name']}] EMERGENCY STOP TRIGGERED!")
            robot_dict['emergency_stop'] = True


def update_slow_mode(robot_dict, joy_msg, logger):
    """
    L2/R2トリガーによるSlow Mode (微修正モード) の判定
    Why: PS4コントローラーのL2/R2トリガーは未入力値が 1.0 であり、
         押し込むと -1.0 に向かって値が小さくなるため、
         PS4_TRIGGER_THRESHOLD (-0.5) 未満の条件で押下と判定する。
    """
    prev_slow = robot_dict['slow_mode']
    robot_dict['slow_mode'] = False

    axis_l2 = get_axis(joy_msg, AXIS_L2)
    axis_r2 = get_axis(joy_msg, AXIS_R2)

    if axis_l2 < PS4_TRIGGER_THRESHOLD or axis_r2 < PS4_TRIGGER_THRESHOLD:
        robot_dict['slow_mode'] = True

    if prev_slow != robot_dict['slow_mode']:
        mode_str = f"SLOW ({int(SLOW_MODE_RATIO * 100)}%)" if robot_dict['slow_mode'] else "NORMAL (100%)"
        logger.info(f"[{robot_dict['name']}] Speed Mode: {mode_str}")


def update_robot_velocity(robot_dict, joy_msg, logger):
    """
    Joy入力をもとに状態更新および速度計算を行うメインエントリー関数
    処理の流れ: ボタン処理 -> Slow Mode処理 -> 速度計算
    """
    # 1. ボタン・緊急停止処理
    update_robot_buttons(robot_dict, joy_msg, logger)

    if robot_dict['emergency_stop']:
        robot_dict['target_vel'] = [0, 0, 0]
        return

    # 2. Slow Mode判定
    update_slow_mode(robot_dict, joy_msg, logger)

    # 3. 3軸(X, Y1, Y2)の目標速度計算
    speed_limit = MAX_VELOCITY * SLOW_MODE_RATIO if robot_dict['slow_mode'] else MAX_VELOCITY

    # 配列要素の順序: 0=X軸, 1=Y1軸, 2=Y2軸
    val_x = apply_deadzone(get_axis(joy_msg, AXIS_LEFT_STICK_X))
    vel_x = int(val_x * speed_limit * robot_dict['invert'][0])

    val_y1 = apply_deadzone(get_axis(joy_msg, AXIS_LEFT_STICK_Y))
    vel_y1 = int(val_y1 * speed_limit * robot_dict['invert'][1])

    val_y2 = apply_deadzone(get_axis(joy_msg, AXIS_RIGHT_STICK_Y))
    vel_y2 = int(val_y2 * speed_limit * robot_dict['invert'][2])

    robot_dict['target_vel'] = [vel_x, vel_y1, vel_y2]

    # スティック入力がある場合にデバッグログを出力
    if val_x != 0.0 or val_y1 != 0.0 or val_y2 != 0.0:
        logger.debug(f"[{robot_dict['name']}] Target velocity: {robot_dict['target_vel']}")


# --- Dynamixel 初期化ヘルパー ---

def init_dynamixel_motor(port_handler, packet_handler, dxl_id, logger=None):
    """
    Dynamixel 1台の初期化 (トルクOFF -> Velocity Mode設定 -> トルクON)
    通信エラーやDynamixelのエラーレスポンスをチェックし、発生時はログを出力する。
    """
    # トルクOFF
    comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if comm_result != 0 or dxl_error != 0:
        msg = f"[Dynamixel ID={dxl_id}] Failed to disable torque: {packet_handler.getTxRxResult(comm_result)}"
        if logger:
            logger.error(msg)
        else:
            print(f"[ERROR] {msg}")
        return False

    # 動作モードをVelocity Control Modeに変更
    comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
    if comm_result != 0 or dxl_error != 0:
        msg = f"[Dynamixel ID={dxl_id}] Failed to write operating mode: {packet_handler.getTxRxResult(comm_result)}"
        if logger:
            logger.error(msg)
        else:
            print(f"[ERROR] {msg}")
        return False

    # トルクON
    comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if comm_result != 0 or dxl_error != 0:
        msg = f"[Dynamixel ID={dxl_id}] Failed to enable torque: {packet_handler.getTxRxResult(comm_result)}"
        if logger:
            logger.error(msg)
        else:
            print(f"[ERROR] {msg}")
        return False

    return True


# --- ROS 2 Node クラス ---

class DualRobotController(Node):
    def __init__(self, port_handler, packet_handler, group_sync_write):
        super().__init__('dual_robot_controller')
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.group_sync_write = group_sync_write
        self.last_warn_time = 0.0

        # 重複callbackを解消し、共通処理 joy_callback を使用
        self.sub_a = self.create_subscription(
            Joy, robot_a['joy_topic'],
            partial(self.joy_callback, robot_a), 10)
        self.sub_b = self.create_subscription(
            Joy, robot_b['joy_topic'],
            partial(self.joy_callback, robot_b), 10)

        # 送信タイマー (20Hz / 0.05秒周期)
        self.timer = self.create_timer(COMM_TIMER_PERIOD, self.timer_callback)

        # Joy接続監視タイマー (10Hz / 0.1秒周期で高速にタイムアウト検知)
        self.health_check_timer = self.create_timer(HEALTH_CHECK_PERIOD, self.check_joy_health)

        self.get_logger().info("=== Dual Robot Controller Initialized (6 Dynamixels) ===")

    def joy_callback(self, robot_dict, msg):
        """
        共通Joy受信Callback
        受信時刻の更新、初回接続 / タイムアウト復帰ログの出力を行う。
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # 初回受信時の接続ログ
        if not robot_dict['joy_connected']:
            self.get_logger().info(f"[{robot_dict['name']}] Joy connected")
            robot_dict['joy_connected'] = True

        # タイムアウト状態からの復帰ログ
        if robot_dict['is_timed_out']:
            self.get_logger().info(f"[{robot_dict['name']}] Joy reconnected")
            robot_dict['is_timed_out'] = False

        robot_dict['last_joy_time'] = now_sec
        update_robot_velocity(robot_dict, msg, self.get_logger())

    def check_joy_health(self):
        """
        Joyコントローラーの受信用タイムアウト監視処理
        一定時間Joyが来ない場合にロボットを停止させる。
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9

        for robot_dict in [robot_a, robot_b]:
            # 1. 一度も受信していない場合の定期案内ログ (3秒ごと)
            if not robot_dict['joy_connected']:
                if now_sec - self.last_warn_time > 3.0:
                    self.get_logger().warn(
                        f"[{robot_dict['name']}] Waiting for initial Joy signal on ({robot_dict['joy_topic']})..."
                    )
                    self.last_warn_time = now_sec
                continue

            # 2. 受信途絶 (タイムアウト) の判定
            if robot_dict['last_joy_time'] is not None:
                elapsed = now_sec - robot_dict['last_joy_time']

                # タイムアウト発生した瞬間
                if elapsed > JOY_TIMEOUT and not robot_dict['is_timed_out']:
                    robot_dict['is_timed_out'] = True
                    robot_dict['target_vel'] = [0, 0, 0]  # 安全のため速度を0に固定
                    self.get_logger().warn(
                        f"[{robot_dict['name']}] Joy timeout ({elapsed:.2f} sec): stopping robot"
                    )
                # タイムアウト継続中
                elif robot_dict['is_timed_out']:
                    robot_dict['target_vel'] = [0, 0, 0]

    def timer_callback(self):
        """
        Dynamixel SDK GroupSyncWrite を使用して 6軸一括送信する処理 (20Hz)
        """
        self.group_sync_write.clearParam()

        for robot_dict in [robot_a, robot_b]:
            for dxl_id, vel in zip(robot_dict['dxl_ids'], robot_dict['target_vel']):
                # Why: Dynamixel SDKへ32bitの速度値を4バイト（リトルエンディアン）に分解して渡す。
                vel_bytes = [(vel >> (8 * i)) & 0xFF for i in range(4)]
                add_param_result = self.group_sync_write.addParam(dxl_id, vel_bytes)
                if not add_param_result:
                    self.get_logger().error(f"[Dynamixel ID={dxl_id}] Failed to add param to GroupSyncWrite")

        # 6軸一括送信
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != 0:
            self.get_logger().error(
                f"GroupSyncWrite txPacket failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )


def main(args=None):
    rclpy.init(args=args)

    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(2.0)
    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

    # 1. シリアルポートのオープン
    if not port_handler.openPort():
        print(f"[FATAL ERROR] Failed to open serial port: {DEVICENAME}")
        sys.exit(1)

    # 2. ボーレートの設定
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"[FATAL ERROR] Failed to set baudrate: {BAUDRATE}")
        sys.exit(1)

    all_dxl_ids = robot_a['dxl_ids'] + robot_b['dxl_ids']

    # 3. 6台のDynamixelの初期化とエラーチェック
    for dxl_id in all_dxl_ids:
        if not init_dynamixel_motor(port_handler, packet_handler, dxl_id):
            print(f"[FATAL ERROR] Failed to initialize Dynamixel ID={dxl_id}")
            # 初期化失敗時は安全のため終了処理へ
            port_handler.closePort()
            sys.exit(1)

    # 4. ROS 2 Node生成 & spin
    node = DualRobotController(port_handler, packet_handler, group_sync_write)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Dual Robot Controller Node...')
    finally:
        # 5. 安全な終了処理: 全軸速度0一括送信 -> Torque OFF -> Port Close
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