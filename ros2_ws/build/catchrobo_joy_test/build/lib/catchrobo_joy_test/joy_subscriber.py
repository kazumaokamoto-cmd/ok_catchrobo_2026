#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# PS4コントローラーの軸・ボタン割り当てインデックス
AXIS_LX = 0   # 左スティック左右
AXIS_LY = 1   # 左スティック前後
AXIS_RX = 3   # 右スティック左右
AXIS_RY = 4   # 右スティック前後


def joy_callback(msg):
    """Joyメッセージを受信したときに呼ばれる関数"""
    # 受信した配列から必要な値を取り出す
    lx = msg.axes[AXIS_LX]
    ly = msg.axes[AXIS_LY]
    rx = msg.axes[AXIS_RX]
    ry = msg.axes[AXIS_RY]

    # ボタン状態（配列の長さチェックを入れて安全に取り出す）
    btn_cross = msg.buttons[0] if len(msg.buttons) > 0 else 0
    btn_circle = msg.buttons[1] if len(msg.buttons) > 1 else 0

    # 画面へリアルタイム表示
    print(f"[Joy Input] Left Stick: (X={lx:+.2f}, Y={ly:+.2f}) | Right Stick: (X={rx:+.2f}, Y={ry:+.2f}) | Buttons: [×:{btn_cross}, ○:{btn_circle}]")


def main(args=None):
    rclpy.init(args=args)
    node = Node('joy_test_subscriber')

    # /joy トピックを受信するSubscriberを作成
    node.create_subscription(
            Joy,
        '/joy',
        joy_callback,
        10
    )

    print("--- PS4コントローラー入力確認ノード起動 ---")
    print("/joy トピックを待機中...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
