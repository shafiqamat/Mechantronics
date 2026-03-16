# -*- coding: utf-8 -*-
"""
Created on Thu Feb 19 14:09:17 2026

@author: maxwe
"""
from pyb import Pin
import motor

right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
left_motor = motor.Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)

right_motor.enable()
left_motor.enable()

#pins corresponding to line sensors
center      = Pin(Pin.cpu.B6,mode=Pin.IN)
left        = Pin(Pin.cpu.A7,mode=Pin.IN)
right       = Pin(Pin.cpu.C7,mode=Pin.IN)
fleft       = Pin(Pin.cpu.A6,mode=Pin.IN)
fright      = Pin(Pin.cpu.A9,mode=Pin.IN)

while True:
    #pick sensor to test
    select = input("which sensor would you like to test (1-5) ctrl+c to exit\r\n")
    if int(select) not in {1,2,3,4,5}:
        raise ValueError
    
    #dict of sensors
    index = {'1':fleft,'2':left,'3':center,'4':right,'5':fright}

    #loop through and print sensor values 
    try:
        count = 0
        while True:
            if count % 1000 == 0:
                print (index[select].value())
            count +=1
    #exit on ctrl+c
    except KeyboardInterrupt:
        continue
            