#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool
from rpi_ws281x import PixelStrip, Color

# NeoPixel configuration:
LED_COUNT = 12       # Number of NeoPixels in the strip
LED_PIN = 18        # GPIO pin connected to the NeoPixels (must support PWM)
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800kHz)
LED_DMA = 10        # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 100  # Set brightness level (0-255)
LED_INVERT = False    # True to invert signal logic
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

class NeoPixelNode(Node):
    def __init__(self):
        super().__init__('neopixel_node')

        self.strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

        self.color_sub = self.create_subscription(
            ColorRGBA,
            'neopixel_color',
            self.color_callback,
            10
        )

        self.on_off_srv = self.create_service(
            SetBool,
            'neopixel_on_off',
            self.on_off_callback
        )

        self.is_on = False
        self.current_color = Color(0, 0, 0)  # Initialize to black (off)
        self.update_neopixel()
        self.get_logger().info('NeoPixel node started')



    def on_off_callback(self, request, response):
        self.is_on = request.data
        self.update_neopixel()
        response.success = True
        response.message = f'NeoPixel turned {"on" if self.is_on else "off"}'
        self.get_logger().info(response.message)
        return response

    def update_neopixel(self):
        if self.is_on:
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, )
            self.strip.show()
        else:
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, Color(0, 0, 0))
            self.strip.show()

def main(args=None):
    rclpy.init(args=args)
    neopixel_node = NeoPixelNode()
    rclpy.spin(neopixel_node)
    neopixel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()