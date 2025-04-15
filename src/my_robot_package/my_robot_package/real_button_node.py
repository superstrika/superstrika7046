import RPi.GPIO as GPIO
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

BUTTON_PIN = 17

class RealButtonPublisher(Node):
    def __init__(self):
        super().__init__('real_button_publisher')

        self.publisher_ = self.create_publisher(Bool, 'button_press', 10)
        self.last_state = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.get_logger().info("(🟢 Button node started (manual polling)")
        threading.Thread(target=self.poll_button_loop, daemon=True).start()

    def poll_button_loop(self):
        while True:
            current_state = GPIO.input(BUTTON_PIN)
            if current_state and not self.last_state:
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                self.get_logger().info("🔘 Button pressed!")
            self.last_state = current_state
            time.sleep(0.05)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealButtonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
