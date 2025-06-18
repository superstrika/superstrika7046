#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
import math
from motor import Motor

class MotorsNode(Node):
    def __init__(self, enablePins: list[tuple[int, int]]):
        super().__init__('motors')
        GPIO.setmode(GPIO.BCM)
        self.PWM_RANGE = 1024

        #motor setup:
        self.motors = []
        for pins in enablePins:
            currentMotor = Motor(*pins)
            currentMotor.stopMotor()
            self.motors.append(currentMotor)
        
        self.get_logger().info('Motor loaded successful')
        print("Motor loaded successful")

        #node setup:
        self.vectorSubscriber = self.create_subscription(
            Float32MultiArray,
            'vectorMovement',
            self.vectorMovementCallback,
            10
        )
        
        self.speedSubscriber = self.create_subscription(
            Float32MultiArray,
            'motorSpeed',
            self.motorSpeedCallback,
            11
        )

        self.speedPublisher = self.create_publisher(
            Float32MultiArray,
            'motorSpeed',
            11
        )

    def speedToPWM(self, speed):
        """Convert speed percentage (-100 to 100) to PWM value (0 to 1024)"""
        speed = max(min(speed, 100), -100)
        return int((abs(speed) / 100.0) * self.PWM_RANGE)
    
    def motorSpeedCallback(self, msg):
        for i in range(len(msg.data)):
            speed = msg.data[i]
            if speed == 0:
                self.motors[i].stopMotor()
            else:
                self.motors[i].startMotor((speed > 0), self.speedToPWM(speed))

    def vectorMovementCallback(self, msg):
        try:
            speed = Float32MultiArray()
            speed.data = self.calculate_speed(msg.data[0], msg.data[1], msg.data[2])
            self.speedPublisher.publish(speed)
        except Exception as e:
            self.get_logger().error(f'Error in vectorMovementCallback: {e}')
            print(e)
    
    def calculate_speed(self, angle, magnitude, rotation):
        # Calculate the speed of the motor
        vx = magnitude * math.cos(math.radians(angle - 45))
        vy = magnitude * math.sin(math.radians(angle - 45))
        wheel1_speed = vy + rotation
        wheel2_speed = vx - rotation
        wheel3_speed = vx + rotation
        wheel4_speed = vy - rotation

        return [wheel1_speed*100, wheel2_speed*100, wheel3_speed*100, wheel4_speed*100]
    
    def printSpeed(self, msg):
        # ANSI escape codes for colors and style
        RED = "\033[91m"
        GREEN = "\033[92m"
        CYAN = "\033[96m"
        YELLOW = "\033[93m"
        RESET = "\033[0m"
        BOLD = "\033[1m"

        print(f"""
    {BOLD}{CYAN}         WHEEL SPEEDS{RESET}
        {YELLOW}┌───────────────────┐{RESET}
        {YELLOW}│{RESET}  [1]         [2]  {YELLOW}│{RESET}
        {YELLOW}│{RESET} ({GREEN}{msg.data[0]:03}{RESET})       ({GREEN}{msg.data[1]:03}{RESET}) {YELLOW}│{RESET}
        {YELLOW}│{RESET}    \\       /      {YELLOW}│{RESET}
        {YELLOW}│{RESET}     \\     /       {YELLOW}│{RESET}
        {YELLOW}│{RESET}     /     \\       {YELLOW}│         {RESET}
        {YELLOW}│{RESET}    /       \\      {YELLOW}│{RESET}
        {YELLOW}│{RESET} ({GREEN}{msg.data[3]:03}{RESET})       ({GREEN}{msg.data[2]:03}{RESET}) {YELLOW}│{RESET}
        {YELLOW}│{RESET}  [4]         [3]  {YELLOW}│{RESET}
        {YELLOW}└───────────────────┘{RESET}
    """)

    def __del__(self):
        # Cleanup GPIO on shutdown
        for currentMotor in self.motors:
            currentMotor.stopMotor()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motorPins = [
        (2, 3),
        (17, 27),
        (14, 15),
        (23, 24)
    ]
    motorNode = MotorsNode(motorPins)

    try:
        rclpy.spin(motorNode)
    except KeyboardInterrupt:
        pass
    finally:
        motorNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
