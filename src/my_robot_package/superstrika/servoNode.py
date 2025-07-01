import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from servo import ServoMotor

class ScreenNode(Node):
    def __init__(self, servoPin, freq=50):
        super().__init__("screen")
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

def main(args=None):
    rclpy.init(args=args)
    screenNode = ScreenNode(0x27, (20, 4))

    try:
        rclpy.spin(screenNode)
    except KeyboardInterrupt:
        pass
    finally:
        screenNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
