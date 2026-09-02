import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class AirCylinderController(Node):
    def __init__(self):
        super().__init__('air_cylinder_controller')

        # トピック設定
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Bool, 'air_cylinder_cmd', 10)

        # 状態管理変数
        self.cylinder_state = False   # 現在のシリンダー状態 (False: OFF, True: ON)
        self.prev_button_state = 0   # 前回の○ボタンの押し状態

        # PS4コントローラーの○ボタンのインデックス（環境によって1または2）
        self.CIRCLE_BUTTON_INDEX = 1  

        self.get_logger().info('Air Cylinder Controller Node Started.')

    def joy_callback(self, msg: Joy):
        # 配列長チェック
        if len(msg.buttons) <= self.CIRCLE_BUTTON_INDEX:
            return

        current_button_state = msg.buttons[self.CIRCLE_BUTTON_INDEX]

        # 立ち上がりエッジ検知（前回は0で、今回は1＝押された瞬間）
        if current_button_state == 1 and self.prev_button_state == 0:
            # 状態を反転
            self.cylinder_state = not self.cylinder_state
            
            # トピックの送信
            cmd_msg = Bool()
            cmd_msg.data = self.cylinder_state
            self.publisher.publish(cmd_msg)

            state_str = "ON" if self.cylinder_state else "OFF"
            self.get_logger().info(f'Air Cylinder toggled: {state_str}')

        # ボタンの状態を更新
        self.prev_button_state = current_button_state

def main(args=None):
    rclpy.init(args=args)
    node = AirCylinderController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
