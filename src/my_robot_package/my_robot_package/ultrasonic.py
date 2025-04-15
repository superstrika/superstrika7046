import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from time import sleep, time

class Ultrasonic(Node):

    def __init__(self):
        super().__init__("Ultrasonic_node")

        self.publisher = self.create_publisher(Float32, "ultra_dist", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.printer = self.create_subscription(Float32, "ultra_dist", self.print_dist, 10)

        self.TRIG = 23
        self.ECHO = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
    
    def timer_callback(self):
        GPIO.output(self.TRIG, False)
        sleep(0.0002)

        GPIO.output(self.TRIG, True)
        sleep(0.00001)

        GPIO.output(self.TRIG, False)

        while GPIO.input(self.ECHO) == 0:
            puls_start = time()
        
        while GPIO.input(self.ECHO) == 1:
            puls_end = time()
        
        dist = round(((puls_end-puls_start) * 17150), 2)

        msg = Float32()
        msg.data = dist
        self.publisher.publish(msg)
    
    def print_dist(self, msg):
        self.get_logger().info(f"dist: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Ultrasonic()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()