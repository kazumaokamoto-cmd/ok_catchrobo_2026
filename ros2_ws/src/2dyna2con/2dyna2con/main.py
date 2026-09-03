import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

# --- Dynamixel 定数定義 (Xシリーズ Velocity Control Mode) ---
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104

LEN_GOAL_VELOCITY = 4
VELOCITY_MODE = 1
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

BAUDRATE = 100000
DEVICENAME = '/dev/ttyUSB0'  # 使用環境に合わせて変更してください (/dev/ttyACM0 等)
MAX_VELOCITY = 200            # テスト用の最大速度値 (Dynamixelの制限範囲内で設定)

# --- 設計書に基づくデータ構造 ---
# 今回は1コントローラーにつき1台（ID: 2 と 5）の制御に簡略化
robot_a = {
    'name': 'robot_a',
    'joy_topic': '/joy_a',
    'dxl_ids': [2],           # コントローラーAに対応するDynamixel ID
    'invert': [1],            # 回転方向反転フラグ
    'target_vel': [0],        # 目標速度
}

robot_b = {
    'name': 'robot_b',
    'joy_topic': '/joy_b',
    'dxl_ids': [5],           # コントローラーBに対応するDynamixel ID
    'invert': [1],            # 回転方向反転フラグ
    'target_vel': [0],        # 目標速度
}


# --- 1. ヘルパー関数 ---
def apply_deadzone(val, threshold=0.05):
    """スティックの不感帯処理"""
    if abs(val) < threshold:
        return 0.0
    return val


# --- 2. ロジック処理関数 ---
def update_robot_velocity(robot_dict, joy_msg):
    """受信したJoyメッセージから目標速度を計算し辞書を更新"""
    if len(joy_msg.axes) > 1:
        # 左スティックY軸 (axes[1]) の入力を使用して速度計算
        raw_val = joy_msg.axes[1]
        val = apply_deadzone(raw_val)
        
        # 速度の算出 (反転フラグの適用)
        target_vel = int(val * MAX_VELOCITY * robot_dict['invert'][0])
        robot_dict['target_vel'] = [target_vel]


class DualRobotController(Node):
    def __init__(self, port_handler, packet_handler, group_sync_write):
        super().__init__('dual_robot_controller')
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.group_sync_write = group_sync_write

        # --- 3. コールバック登録 ---
        self.sub_a = self.create_subscription(
            Joy, robot_a['joy_topic'], self.joy_a_callback, 10)
        self.sub_b = self.create_subscription(
            Joy, robot_b['joy_topic'], self.joy_b_callback, 10)

        # --- 4. タイマー登録 (20Hz / 0.05秒周期) ---
        self.timer = self.create_timer(0.05, self.timer_callback)

    def joy_a_callback(self, msg):
        update_robot_velocity(robot_a, msg)

    def joy_b_callback(self, msg):
        update_robot_velocity(robot_b, msg)

    def timer_callback(self):
        """Dynamixel SDKの GroupSyncWrite を使用して2台へ一括送信"""
        self.group_sync_write.clearParam()

        for robot in [robot_a, robot_b]:
            for dxl_id, vel in zip(robot['dxl_ids'], robot['target_vel']):
                # 負の速度値を2の補数表現（4バイト）に変換
                vel_bytes = [(vel >> (8 * i)) & 0xFF for i in range(4)]
                
                # パラメータ追加
                add_param_result = self.group_sync_write.addParam(dxl_id, vel_bytes)
                if not add_param_result:
                    self.get_logger().error(f"Failed to add param for Dynamixel ID {dxl_id}")

        # 一括送信
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != 0:
            self.get_logger().error(
                f"SyncWrite failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")


# --- 5. 全体管理関数 ---
def main(args=None):
    rclpy.init(args=args)

    # 通信ポート・プロトコル初期化
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(2.0)  # Protocol 2.0
    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

    # シリアルポートオープン
    if not port_handler.openPort():
        print(f"Failed to open port: {DEVICENAME}")
        sys.exit(1)

    # ボーレート設定 (100000)
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate: {BAUDRATE}")
        sys.exit(1)

    # 各Dynamixelの動作モード変更 (速度制御モード) と トルクON
    test_ids = robot_a['dxl_ids'] + robot_b['dxl_ids']
    for dxl_id in test_ids:
        # トルクOFF (設定変更のため)
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        # 速度制御モードへ切り替え
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
        # トルクON
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    node = DualRobotController(port_handler, packet_handler, group_sync_write)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        # 終了処理：全軸へ速度0送信 & トルクOFF
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