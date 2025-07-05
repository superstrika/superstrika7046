import RPi.GPIO as GPIO

class Motor:
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
        """Stops all motors"""
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
