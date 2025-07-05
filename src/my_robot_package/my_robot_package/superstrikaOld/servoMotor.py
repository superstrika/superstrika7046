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
        # The original calculation (angle / 18) can push some servos beyond 180 degrees.
        # This new calculation (angle / 20) provides a more conservative range.
        # duty = 2 + (angle / 20)
        self.motor.ChangeDutyCycle(12)
        sleep(0.5)
        self.motor.ChangeDutyCycle(2)
        sleep(0.5)
        self.motor.ChangeDutyCycle(0)
        self.motor.stop()
        print("Done")
        # self.motor.ChangeDutyCycle(0)