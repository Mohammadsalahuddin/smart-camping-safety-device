from gpiozero import Servo
from time import sleep

servo = Servo(17)
while True:
    servo.min()
