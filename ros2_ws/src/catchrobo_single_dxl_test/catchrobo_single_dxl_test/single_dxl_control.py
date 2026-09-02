#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from dynamixel_sdk import PortHandler, PacketHandler

# ==========================================
# 定数・設定パラメータ
# ==========================================
DEVICENAME = '/dev/ttyUSB0'   # U2D2などのシリアルポート名
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

DXL_ID = 1                    # テストするDynamixelのID
ADDR_OPERATING_MODE = 11      # 動作モード指定のアドレス
ADDR_TORQUE_ENABLE = 64      # トルクON/OFFのアドレス
ADDR_GOAL_VELOCITY = 104     # Goal Velocityのアドレス

VELOCITY_MODE = 1             # 速度制御モードの値

AXIS_LY = 1                   # 左スティック前後で制御
DEADZONE = 0.05               # 不感帯
MAX_VELOCITY = 200            # 最大速度値

# 通信ハンドラー（グローバル変数）
port_handler = None
packet_handler = None


def apply_deadzone(value, threshold):
    """不感帯処理"""
    if abs(value) < threshold:
        return 0.0
    return value


def set_dxl_velocity(velocity):
    """Dynamixel 1台へ速度指令を送信（単一書き込み）"""
    # 負の数を32ビットの無符号整数（2の補数表現）に変換
    vel_val = int(velocity) & 0xFFFFFFFF

    # 4バイト書き込み API
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(
        port_handler, DXL_ID, ADDR_GOAL_VELOCITY, vel_val
    )
    if dxl_comm_result != 0:
        print(f"通信エラー: {packet_handler.getTxRxResult(dxl_comm_result)}")


def joy_callback(msg):
    """Joyトピックを受信したときの処理"""
    if len(msg.axes) <= AXIS_LY:
        return

    # 左スティック前後の値を取得
    raw_ly = msg.axes[AXIS_LY]

    # 不感帯処理を適用
    ly = apply_deadzone(raw_ly, DEADZONE)

    # 目標速度の計算（スティック前でプラス方向）
    target_velocity = ly * MAX_VELOCITY

    # Dynamixelへ直接速度命令を発行
    set_dxl_velocity(target_velocity)
    print(f"[Control] Stick Y: {ly:+.2f} -> Goal Velocity: {target_velocity:+.0f}")


def main(args=None):
    global port_handler, packet_handler

    # 1. Dynamixel 通信初期化
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"ポート {DEVICENAME} を開けませんでした")
        return
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"ボーレート {BAUDRATE} の設定に失敗しました")
        return

    # 動作モードを「速度制御モード」に変更（※トルクOFFの状態で設定）
    packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_OPERATING_MODE, VELOCITY_MODE)

    # トルクのON化
    packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
    print(f"Dynamixel ID:{DXL_ID} 通信確立・速度制御モード設定・トルクON成功")

    # 2. ROS 2 ノード初期化
    rclpy.init(args=args)
    node = Node('single_dxl_test_node')

    node.create_subscription(Joy, '/joy', joy_callback, 10)
    print("--- PS4コントローラー → Dynamixel 1軸テスト起動 ---")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 安全停止処理（速度0 ＆ トルクOFF）
        set_dxl_velocity(0)
        packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        print("トルクOFFおよびポートクローズ完了")

        port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
