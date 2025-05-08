import pigpio
from time import sleep

pi = pigpio.pi()

pi.set_mode(12, pigpio.INPUT)  # GPIO  4 as input
while True:
    print(pi.read(17))
    sleep(.5)