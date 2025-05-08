# button_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ButtonListener(Node):
    def __init__(self):
        super().__init__('button_listener')
        self.subscription = self.create_subscription(Bool, 'button_press', self.button_callback, 10)
        self.get_logger().info('Listening for button press...')

    def button_callback(self, msg):
        if msg.data:
            self.get_logger().info('🔘 Button was clicked!')

def main(args=None):
    rclpy.init(args=args)
    node = ButtonListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
