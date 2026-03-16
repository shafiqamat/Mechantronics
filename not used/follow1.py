# -*- coding: utf-8 -*-
"""
Created on Wed Feb 25 20:17:01 2026

@author: maxwe
"""
from pyb import Pin
import motor
import linesen

right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
left_motor = motor.Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)

right_motor.enable()
left_motor.enable()

sens = linesen.sensor(Pin.cpu.A6,Pin.cpu.A7,
                          Pin.cpu.B6,
                          Pin.cpu.C7,Pin.cpu.A9)

Kp = 15
Ki = 5
speed = 20


while True:
    right_motor.set_effort(0)
    left_motor.set_effort(0)
    totErr = 0.0
    print(sens.get_pos())
    userin = input('Enter anything to start\r\n')

    try:
        while True:
            error = sens.get_pos()
            
            
            #set base speeds
            leff = speed
            reff = speed
            #accumulate error 
            totErr += error * 0.05
            #do preportional controll
            error = error * Kp
            #do intergral control
            error += totErr * Ki
            #finally set effort
            leff -= error  
            reff += error
            left_motor.set_effort(leff)
            right_motor.set_effort(reff)
    except KeyboardInterrupt:
        continue