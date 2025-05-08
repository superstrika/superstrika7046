import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
import RPi.GPIO as GPIO
from time import sleep, time
from RPLCD.i2c import CharLCD
import requests

class Pot_screen(Node):
    
    def __init__(self):
        super().__init__("Pot_screen")
        self.publisher = self.create_publisher(String, "Pot", 10)
        self.timer_ = self.create_timer(2, self.check_pot)

        self.lcd = CharLCD(
            i2c_expander='PCF8574',
            address=0x27,
            port=1,
            cols=20,
            rows=4,
            charmap='A00',
            auto_linebreaks=True,
            backlight_enabled=True
        )

        self.screen = self.create_subscription(String, "Pot", self.print_pot, 10)
        
    
    def check_pot(self):
       URL = "http://192.168.87.23/pot"
       resp = requests.get(URL)
       
       msg = String()
       pot_value = resp.text.split(" ")[1]
       msg.data = pot_value
       self.publisher.publish(msg)
       self.get_logger().info(f"pot value: {pot_value}")
    
    def print_pot(self, msg):
        self.lcd.clear()
        self.lcd.cursor_pos = (1, 0)
        self.lcd.write_string(f"pot: {msg.data}")
    
    



def main(args=None):
   rclpy.init(args=args)
   node = Pot_screen()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()

if __name__ == "__main__":
   main()