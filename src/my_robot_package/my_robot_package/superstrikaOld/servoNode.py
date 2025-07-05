import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from my_robot_package.superstrika.servoMotor import ServoMotor

class ServoNode(Node):
    def __init__(self, servoPin, freq=50):
        super().__init__("screen")
        GPIO.setmode(GPIO.BCM)
        self.DEFAULT_ANGLE = 0
        
        self.motor = ServoMotor(servoPin, freq)

        self.servoSubscription = self.create_subscription(
            Float32,
            "servoAngle",
            self.changeAngle,
            13
        )

    def changeAngle(self, msg):
        self.motor.setMotorAngle(msg.data)

        print(f"Set servo angle to {msg.data}")
        self.get_logger().info(f"Set servo angle to {msg.data}")
    
    def __del__(self):
        self.motor.setMotorAngle(self.DEFAULT_ANGLE)
        GPIO.cleanup()        

def main(args=None):
    rclpy.init(args=args)
    servoNode = ServoNode(14)

    try:
        rclpy.spin(servoNode)
    except KeyboardInterrupt:
        pass
    finally:
        servoNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
