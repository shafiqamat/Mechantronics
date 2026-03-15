# -*- coding: utf-8 -*-
"""
Created on Wed Feb 25 18:45:39 2026

@author: maxwe
"""

from pyb import Pin

class sensor():
    '''creates a class that can get the current line position in terms of a 
    5 IR sensor array'''
    
    def __init__(self,pin1,pin2,pin3,pin4,pin5):
        
        
        self.center = Pin(pin3, mode=Pin.IN)
        self.left   = Pin(pin2, mode=Pin.IN)
        self.right  = Pin(pin4, mode=Pin.IN)
        self.fleft  = Pin(pin1, mode=Pin.IN)
        self.fright = Pin(pin5, mode=Pin.IN)
        
        
    def get_pos(self):
        '''returns the current position of the line sensor, where zero position
        is equivilient to centered, 
        pointing left -> positive,
        pointing right -> negative
        returns for 100 no read'''
        
        pos = 0.0
        n = 0
        if not self.fleft.value():  pos -= 2; n += 1
        if not self.left.value():   pos -= 1; n += 1
        if not self.center.value(): pos += 0; n += 1
        if not self.right.value():  pos += 1; n += 1
        if not self.fright.value(): pos += 2; n += 1
        if n == 0:
            return 100
        return pos / n