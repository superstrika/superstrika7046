import RPi.GPIO as GPIO
from time import sleep

class ServoMotor:
    def __init__(self, servoPin, frequency=50):
        self.servoPin = servoPin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPin, GPIO.OUT)

        self.motor = GPIO.PWM(servoPin, frequency)
        self.motor.start(0)        
    
    def setMotorAngle(self, angle):
        duty = 2 + (angle / 18)
        GPIO.output(self.servoPin, True)
        self.motor.ChangeDutyCycle(duty)
        sleep(0.5)
        GPIO.output(self.servoPin, False)
        self.motor.ChangeDutyCycle(0)