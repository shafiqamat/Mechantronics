# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 14:06:11 2026

@author: maxwe
"""
import encoder
from pyb import Pin

RE = encoder.Encoder(2,Pin.cpu.A0,1,Pin.cpu.A1,2)
RE.zero()
count = 0
while True:
    RE.update()
    if count % 100 == 0:
        print(RE.get_position())
    count += 1

    