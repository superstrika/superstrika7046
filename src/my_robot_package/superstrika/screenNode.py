import rclpy
from rclpy.node import Node
from std_msgs.msg import StringMultiArray
from RPLCD.i2c import CharLCD

class ScreenNode(Node):
    def __init__(self, address, size: tuple):
        super().__init__("screen")
        self.size: tuple = size

        self.screenSubscription = self.create_subscription(
            StringMultiArray,
            "printScreen",
            self.printScreen,
            12
        )

        self.screen = CharLCD(
            i2c_expander='PCF8574',
            address=address,
            port=1,
            cols=self.size[0],
            rows=self.size[1],
            charmap='A00',
            auto_linebreaks=True,
            backlight_enabled=True
        )
    
    def printScreen(self, msg):
        self.screen.clear()
        for i in range(len(msg.data)):
            if i < self.size[1] and len(msg.data[i]) < self.size[0]:
                self.screen.cursor_pos = (i, 0)
                self.screen.write_string(msg.data[i])
    
    def __del__(self):
        self.screen.clear()

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
