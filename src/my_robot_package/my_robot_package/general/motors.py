#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
import math

class motor():
    def __init__(self, pin1, pin2, frequency=2000):
        self.PWM_RANGE = 1024

        GPIO.setup(pin1, GPIO.OUT)
        GPIO.setup(pin2, GPIO.OUT)

        self.pwm1 = GPIO.PWM(pin1, frequency)
        self.pwm2 = GPIO.PWM(pin2, frequency)

        self.pwm1.start(0)
        self.pwm2.start(0)
    
    def startMotor(self, direction, cycle):
        """
        cycle - 0-1024.
        direction - true: forward, direction - false: backward.
        """
        if direction:
            self.pwm1.ChangeDutyCycle((cycle / self.PWM_RANGE) * 100)
            self.pwm2.ChangeDutyCycle(0)
        else:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle((cycle / self.PWM_RANGE) * 100)
    
    def stopMotor(self):
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)



class MotorNode(Node):
    def __init__(self, enablePins: list[tuple[int, int]]):
        super().__init__('motors')
        GPIO.setmode(GPIO.BCM)
        
        self.motors = []
        self.enablePins = enablePins

        for pins in enablePins:
            currentMotor = motor(*pins)
            currentMotor.stopMotor()
            self.motors.append(currentMotor)

        self.get_logger().info('Motor loaded successful')
        print("Motor loaded successful")

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
        
        # Create a timer to publish motor speeds periodically
        self.timer = self.create_timer(2.5, self.publish_motor_speed)  # 10 Hz
        
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
        for i in range(msg.data):
            # Get the first motor speed value
            speed = msg.data[i]
            
            # Convert speed to PWM value
            pwm_value = self.convert_to_pwm_value(speed)
            
            if speed == 0:
                self.motors[i].stopMotor()
            else:
                self.motors[i].startMotor((speed > 0), pwm_value)

            print(f'Setting motor {i+1} speed to: {speed}%, PWM value: {pwm_value}, Duty cycle: {(pwm_value / self.PWM_RANGE) * 100:.1f}%')
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