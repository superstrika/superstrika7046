#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
import math

class MotorNode(Node):
    def __init__(self):
        super().__init__('motors')
        
        # Create publisher for motor speeds
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'motor_speed',
            10)
        
        # Create subscriber for motor speeds
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speed',
            self.motor_callback,
            10)
        
        # Setup GPIO for ZK-5AD motor driver
        GPIO.setmode(GPIO.BCM)
        
        # Motor 1 pins (GPIO 2 and 3)
        self.motor1_pin1 = 2  # First PWM pin for motor 1
        self.motor1_pin2 = 3  # Second PWM pin for motor 1
        
        # Setup GPIO pins as outputs
        GPIO.setup(self.motor1_pin1, GPIO.OUT)
        GPIO.setup(self.motor1_pin2, GPIO.OUT)
        
        # Create PWM instances with higher frequency for better control
        self.pwm1_1 = GPIO.PWM(self.motor1_pin1, 2000)  # 2000 Hz frequency
        self.pwm1_2 = GPIO.PWM(self.motor1_pin2, 2000)  # 2000 Hz frequency
        
        # Start PWM with 0% duty cycle
        self.pwm1_1.start(0)
        self.pwm1_2.start(0)
        
        # PWM range for ESP32 compatibility (0-1024)
        self.PWM_RANGE = 1024
        
        self.get_logger().info('Motor node started')
        
        # Create a timer to publish motor speeds periodically
        self.timer = self.create_timer(0.1, self.publish_motor_speed)  # 10 Hz
        
        # Initialize motor speed
        self.current_speed = 0.0

        # Initialize motor speed
        self.angle = 180
        self.magnitude = 0.8
        self.rotation = 0.1
    
    def convert_to_pwm_value(self, speed_percent):
        """Convert speed percentage (-100 to 100) to PWM value (0 to 1024)"""
        # Ensure speed is between -100 and 100
        speed_percent = max(min(speed_percent, 100), -100)
        
        # Convert directly to PWM range (0-1024)
        # For example: 30% speed = 0.3 * 1024 = 307
        pwm_value = int((abs(speed_percent) / 100.0) * self.PWM_RANGE)
        
        return pwm_value
    
    def motor_callback(self, msg):
        if len(msg.data) > 0:
            # Get the first motor speed value
            speed = msg.data[0]
            
            # Update current speed
            self.current_speed = speed
            
            # Convert speed to PWM value
            pwm_value = self.convert_to_pwm_value(speed)
            
            # Control motor based on speed
            if speed > 0:  # Forward
                self.pwm1_1.ChangeDutyCycle((pwm_value / self.PWM_RANGE) * 100)
                self.pwm1_2.ChangeDutyCycle(0)
            elif speed < 0:  # Reverse
                self.pwm1_1.ChangeDutyCycle(0)
                self.pwm1_2.ChangeDutyCycle((pwm_value / self.PWM_RANGE) * 100)
            else:  # Stop
                self.pwm1_1.ChangeDutyCycle(0)
                self.pwm1_2.ChangeDutyCycle(0)
            
            self.get_logger().info(f'Setting motor speed to: {speed}%, PWM value: {pwm_value}, Duty cycle: {(pwm_value / self.PWM_RANGE) * 100:.1f}%')
    
    def calculate_speed(self, angle, magnitude, rotation):
        # Calculate the speed of the motor
        wheel1_speed = magnitude * math.cos(angle) + rotation
        wheel2_speed = magnitude * math.sin(angle) + rotation
        wheel3_speed = magnitude * math.cos(angle) - rotation
        wheel4_speed = magnitude * math.sin(angle) - rotation
        speed = [wheel1_speed*100, wheel2_speed*100, wheel3_speed*100, wheel4_speed*100]
        return speed

    def publish_motor_speed(self):
        # Create and publish current motor speed
        msg = Float32MultiArray()
        msg.data = self.calculate_speed(self.angle, self.magnitude, self.rotation)
        self.publisher.publish(msg)
    
    def __del__(self):
        # Cleanup GPIO on shutdown
        self.pwm1_1.stop()
        self.pwm1_2.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()