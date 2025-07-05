from gpiozero import PWMLED
from time import sleep

class ServoMotor:
    def __init__(self, servoPin):
        self.PWM_RANGE = 1024

        self.servo = PWMLED(servoPin)
        self.servo.value = 0
    
    def setMotorAngle(self, angle):
        self.servo.value = (2 + (angle / 18)) / self.PWM_RANGE
        sleep(0.5)
        self.servo.value = 0

    def stopMotor(self):
        self.servo.value = 0