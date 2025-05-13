#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rpi_ws281x import PixelStrip, Color

class NeoPixelNode(Node):
    def __init__(self):
        super().__init__('tomer_node')

        self.publisher_ = self.create_publisher(Bool, 'button', 10)




def main(args=None):
    print(1)
    rclpy.init(args=args)
    neopixel_node = NeoPixelNode()
    rclpy.spin(neopixel_node)
    neopixel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()