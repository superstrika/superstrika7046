import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
from time import sleep

BUTTON_PIN = 17
LED_PIN = 12

class PrintHello(Node):

    def __init__(self):
        super().__init__("Print_hello")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(LED_PIN, GPIO.OUT)

        self.publisher = self.create_publisher(String, "BUTTON", 10)
        self.timer_ = self.create_timer(0.5, self.publish_hello)
        self.led = self.create_subscription(String, "BUTTON", self.led_output, 10)
    
    def publish_hello(self):
        msg = String()
        msg.data = str(GPIO.input(BUTTON_PIN))
        self.publisher.publish(msg)
    
    def led_output(self, msg):
        self.get_logger().info(f"led state: {msg.data}")
        GPIO.output(LED_PIN, bool(msg.data))
        sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = PrintHello()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
