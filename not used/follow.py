# -*- coding: utf-8 -*-
"""
Created on Fri Feb 20 18:45:08 2026

@author: maxwe
"""
import time
from pyb import Pin
import motor

right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
left_motor = motor.Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)

right_motor.enable()
left_motor.enable()

# Pins corresponding to line sensors
center = Pin(Pin.cpu.B6, mode=Pin.IN)
left = Pin(Pin.cpu.A7, mode=Pin.IN)
right = Pin(Pin.cpu.C7, mode=Pin.IN)
fleft = Pin(Pin.cpu.A6, mode=Pin.IN)
fright = Pin(Pin.cpu.A9, mode=Pin.IN)

reff = 0
leff = 0

# Driving loop
while True:
    right_motor.set_effort(0)
    left_motor.set_effort(0)
    userin = input('Enter anything to start\r\n')

    try:
        while True:
            if not center.value():
                reff += 20
                leff += 20
            if not left.value():
                reff += 30
                leff += 10
            if not right.value():
                reff += 10
                leff += 30
            if not fleft.value():
                leff -= 20
            if not fright.value():
                reff -= 20
            right_motor.set_effort(reff)
            left_motor.set_effort(leff)
            time.sleep_ms(30)
            leff = 20
            reff = 20
    except KeyboardInterrupt:
        continue
