#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# Define GPIO pins for trigger and echo
TRIGGER_PIN = 23
ECHO_PIN = 24

# Define speed of sound in cm/s (approximately)
SPEED_OF_SOUND = 34300

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(0.1, self.measure_distance)  # Measure every 100ms

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)

        GPIO.output(TRIGGER_PIN, False)
        self.get_logger().info('Ultrasonic node started')

    def trigger_sensor(self):
        # Send a short trigger pulse
        GPIO.output(TRIGGER_PIN, True)
        time.sleep(0.00001)  # 10 microsecond pulse
        GPIO.output(TRIGGER_PIN, False)

    def measure_distance(self):
        self.trigger_sensor()

        pulse_start_time = time.time()
        pulse_end_time = time.time()

        # Wait for the echo pin to go high (with a timeout)
        max_wait = 0.1  # Maximum time to wait for echo in seconds
        start_time = time.time()
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start_time = time.time()
            if (time.time() - start_time) > max_wait:
                self.get_logger().warn('Echo signal not received (start)')
                return

        # Wait for the echo pin to go low (with a timeout)
        start_time = time.time()
        while GPIO.input(ECHO_PIN) == 1:
            pulse_end_time = time.time()
            if (time.time() - start_time) > max_wait:
                self.get_logger().warn('Echo signal not received (end)')
                return

        pulse_duration = pulse_end_time - pulse_start_time
        distance_cm = (pulse_duration * SPEED_OF_SOUND) / 2
        self.get_logger().info(f'Distance: {distance_cm:.2f} cm')

        msg = Float32()
        msg.data = distance_cm
        self.publisher_.publish(msg)

    def on_shutdown(self):
        GPIO.cleanup()
        self.get_logger().info('Ultrasonic node shutting down and cleaning up GPIO')

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_node = UltrasonicNode()
    rclpy.spin(ultrasonic_node)
    ultrasonic_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup() # Ensure GPIO cleanup even if Ctrl+C is missed during spin

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Ultrasonic node interrupted by user')
    finally:
        GPIO.cleanup()