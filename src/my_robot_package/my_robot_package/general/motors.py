#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
import math

class MotorNode(Node):
    def __init__(self, enablePins: list[tuple[int, int]]):
        super().__init__('motors')
        GPIO.setmode(GPIO.BCM)
        self.PWM_RANGE = 1024

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
        
        # self.vectorPublisher = self.create_publisher(
        #     Float32MultiArray,
        #     'vectorMovement',
        #     12
        # )

        self.vectorSubscriber = self.create_subscription(
            Float32MultiArray,
            'vectorMovement',
            self.publish_motor_speed,
            12
        )

        # Create a timer to publish motor speeds periodically
        # self.timer = self.create_timer(2.5, self.publish_motor_speed)  # 10 Hz
        
        # Initialize motor speed
        self.current_speed = 0.0

        # Initialize motor speed
        self.angle = 0
        self.magnitude = 1
        self.rotation = 1.8

    def convert_to_pwm_value(self, speed_percent):
        """Convert speed percentage (-100 to 100) to PWM value (0 to 1024)"""
        # Ensure speed is between -100 and 100
        speed_percent = max(min(speed_percent, 100), -100)
        
        # Convert directly to PWM range (0-1024)
        # For example: 30% speed = 0.3 * 1024 = 307
        pwm_value = int((abs(speed_percent) / 100.0) * self.PWM_RANGE)
        
        return pwm_value
    
    def motor_callback(self, msg):
        for i in range(len(msg.data)):
            # Get the first motor speed value
            speed = msg.data[i]
            
            # Convert speed to PWM value
            pwm_value = self.convert_to_pwm_value(speed)
            
            if speed == 0:
                self.motors[i].stopMotor()
            else:
                self.motors[i].startMotor((speed > 0), pwm_value)

            self.printSpeed(msg)
            print(f'Setting motor {i+1} speed to: {speed}%, PWM value: {pwm_value}, Duty cycle: {(pwm_value / self.PWM_RANGE) * 100:.1f}%')
            self.get_logger().info(f'Setting motor speed to: {speed}%, PWM value: {pwm_value}, Duty cycle: {(pwm_value / self.PWM_RANGE) * 100:.1f}%')
    
    def calculate_speed(self, angle, magnitude, rotation):
        # Calculate the speed of the motor
        vx = magnitude * math.cos(math.radians(angle - 45))
        vy = magnitude * math.sin(math.radians(angle - 45))
        wheel1_speed = vy + rotation
        wheel2_speed = vx - rotation
        wheel3_speed = vx + rotation
        wheel4_speed = vy - rotation
        speed = [wheel1_speed*100, wheel2_speed*100, wheel3_speed*100, wheel4_speed*100]
        return speed

    def publish_motor_speed(self, msg):
        # Create and publish current motor speed
        try:
            speed = Float32MultiArray()
            speed.data = self.calculate_speed(msg.data[0], msg.data[1], msg.data[2])
            self.publisher.publish(speed)
            print(speed.data[0])
            print(speed.data[1])
            print(speed.data[2])
            print("hi")
        except Exception as e:
            print(e)
    
    def printSpeed(self, msg):
        print("""
   #######%03d########
   ##################         
   ####%03d####%03d####
   ##################         
   #######%03d########
    """ % (msg.data[0], msg.data[1], msg.data[2], msg.data[3]))

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
    motor_node = MotorNode(motorPins)
    
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()