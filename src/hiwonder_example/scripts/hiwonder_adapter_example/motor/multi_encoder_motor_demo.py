#!/usr/bin/python3
# coding=utf8
import sys
import time
sys.path.append("..")
from sdk import encoder_motor

my_motor = encoder_motor.EncoderMotorController(1)
print('2 motor forward and reverse')

while True:
    try:
        speed = [0, 20, 0, 20]
        my_motor.set_speed(speed)
        print('\r<---', end='', flush=True)
        time.sleep(1)
        speed = [0, -20, 0, -20]
        my_motor.set_speed(speed)
        print('\r--->', end='', flush=True)
        time.sleep(1)
    except KeyboardInterrupt:
        my_motor.set_speed([0, 0, 0, 0])
        break
