#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speed',
            self.motor_callback,
            10)
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        self.motor_pin = 18  # GPIO pin for motor control
        GPIO.setup(self.motor_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.motor_pin, 1000)  # 1000 Hz frequency
        self.pwm.start(0)  # Start with 0% duty cycle
        
        self.get_logger().info('Motor controller node started')
    
    def motor_callback(self, msg):
        if len(msg.data) > 0:
            # Get the first motor speed value (since we have only one motor)
            speed = msg.data[0]
            
            # Ensure speed is between -100 and 100
            speed = max(min(speed, 100), -100)
            
            # Convert speed to duty cycle (0-100)
            duty_cycle = abs(speed)
            
            # Set the motor speed
            self.pwm.ChangeDutyCycle(duty_cycle)
            
            self.get_logger().info(f'Setting motor speed to: {speed}%')
    
    def __del__(self):
        # Cleanup GPIO on shutdown
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 