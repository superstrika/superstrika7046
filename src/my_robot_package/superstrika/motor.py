from gpiozero import PWMLED

class Motor:
    def __init__(self, pin1, pin2, frequency=2000):
        self.PWM_RANGE = 1024

        self.pwm1 = PWMLED(pin1)
        self.pwm2 = PWMLED(pin2)

        self.pwm1.value = 0
        self.pwm2.value = 0
    
    def startMotor(self, direction, cycle):
        """
        cycle - 0-1024.
        direction - true: forward, direction - false: backward.
        """
        if direction:
            self.pwm1.value = cycle / self.PWM_RANGE
            self.pwm2.value = 0
        else:
            self.pwm1.value = 0
            self.pwm2.value = cycle / self.PWM_RANGE
    
    def stopMotor(self):
        """Stops all motors"""
        self.pwm1.value = 0
        self.pwm2.value = 0
