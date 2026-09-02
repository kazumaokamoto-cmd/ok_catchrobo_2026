#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite

# --- Dynamixel & 通信設定 ---
DEVICENAME = '/dev/ttyUSB0'        # 環境に合わせて変更してください（例: /dev/ttyUSB0）
BAUDRATE = 115200
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64            # Torque Enable (1: Enable, 0: Disable)
ADDR_OPERATING_MODE = 11           # Operating Mode (1: Velocity Control Mode)
ADDR_GOAL_VELOCITY = 104           # Goal Velocity (4 bytes)
LEN_GOAL_VELOCITY = 4

MAX_VELOCITY = 200                 # テスト用の最高速度値

# --- 制御用データ構造（テスト用：各1軸構成） ---
robot_a = {
    'name': 'robot_a',
    'joy_topic': '/joy_a',
    'dxl_ids': [2],                # X軸のみ（ID: 1）
    'invert': [1],
    'target_vel': [0],
    'slow_mode': False,
}

robot_b = {
    'name': 'robot_b',
    'joy_topic': '/joy_b',
    'dxl_ids': [5],                # X軸のみ（ID: 4）
    'invert': [1],
    'target_vel': [0],
    'slow_mode': False,
}

# --- 1. ヘルパー関数 ---
def apply_deadzone(val: float, threshold: float = 0.05) -> float:
    """スティックの不感帯処理"""
    if abs(val) < threshold:
        return 0.0
    return val

# --- 2. ロジック処理関数 ---
def update_robot_velocity(robot_dict: dict, joy_msg: Joy):
    """コントローラー入力から目標速度を計算し辞書を更新"""
    # L2(axes[2]) / R2(axes[5]) トリガーの押し込み判定 (初期値1.0, 押し込むと-1.0)
    # どちらかが半分以上押し込まれていれば微修正モード(50%減衰)
    l2_pressed = len(joy_msg.axes) > 2 and joy_msg.axes[2] < 0.0
    r2_pressed = len(joy_msg.axes) > 5 and joy_msg.axes[5] < 0.0
    robot_dict['slow_mode'] = l2_pressed or r2_pressed

    speed_factor = 0.5 if robot_dict['slow_mode'] else 1.0

    # 左スティック X軸 (axes[0])
    raw_x = joy_msg.axes[0] if len(joy_msg.axes) > 0 else 0.0
    x_val = apply_deadzone(raw_x)

    # 速度計算 (反転フラグ・倍率の適用)
    calc_vx = int(x_val * MAX_VELOCITY * robot_dict['invert'][0] * speed_factor)
    
    robot_dict['target_vel'] = [calc_vx]


class DualRobotTestNode(Node):
    def __init__(self, port_handler, packet_handler, group_sync_write):
        super().__init__('dual_robot_test_node')
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.group_sync_write = group_sync_write

        # --- 3. コールバック登録 ---
        self.sub_joy_a = self.create_subscription(
            Joy, robot_a['joy_topic'], self.joy_a_callback, 10)
        self.sub_joy_b = self.create_subscription(
            Joy, robot_b['joy_topic'], self.joy_b_callback, 10)

        # --- 4. タイマー登録 (20Hz / 0.05秒間隔) ---
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Dual Robot Test Node Started.')

    def joy_a_callback(self, msg: Joy):
        """/joy_a 受信ハンドラ"""
        update_robot_velocity(robot_a, msg)

    def joy_b_callback(self, msg: Joy):
        """/joy_b 受信ハンドラ"""
        update_robot_velocity(robot_b, msg)

    def timer_callback(self):
        """送信タイマーハンドラ（2軸まとめて同期送信）"""
        self.group_sync_write.clearParam()

        # robot_a と robot_b の通信パラメータ設定
        for robot in [robot_a, robot_b]:
            for dxl_id, vel in zip(robot['dxl_ids'], robot['target_vel']):
                # 4バイトの32ビット整数（符号付き）をパケット用バイト配列に変換
                param_goal_velocity = [
                    (int(vel) >> 0) & 0xFF,
                    (int(vel) >> 8) & 0xFF,
                    (int(vel) >> 16) & 0xFF,
                    (int(vel) >> 24) & 0xFF
                ]
                self.group_sync_write.addParam(dxl_id, param_goal_velocity)

        # パケット一括送信
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != 0:
            self.get_logger().warn(f"Failed to send velocity commands: {self.packet_handler.getTxRxResult(dxl_comm_result)}")


# --- 5. 全体管理（メイン処理） ---
def main(args=None):
    rclpy.init(args=args)

    # Dynamixel ポート・パケットハンドラの初期化
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"Failed to open port: {DEVICENAME}")
        sys.exit(1)

    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate: {BAUDRATE}")
        sys.exit(1)

    # テスト対象のDynamixel IDリスト [ID 1, ID 4]
    target_ids = robot_a['dxl_ids'] + robot_b['dxl_ids']

    # 動作モード変更およびトルクON処理
    for dxl_id in target_ids:
        # トルクOFF（動作モード変更に必須）
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        # 速度制御モード(1)に設定
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, 1)
        # トルクON
        packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1)

    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

    node = DualRobotTestNode(port_handler, packet_handler, group_sync_write)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down... Stopping Dynamixels.')
    finally:
        # --- 安全停止処理 ---
        group_sync_write.clearParam()
        stop_bytes = [0, 0, 0, 0]
        for dxl_id in target_ids:
            group_sync_write.addParam(dxl_id, stop_bytes)
        group_sync_write.txPacket()

        # トルクOFF
        for dxl_id in target_ids:
            packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)

        port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
