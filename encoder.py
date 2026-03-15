# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 12:26:59 2026

@author: maxwe
"""
from pyb import Timer
from time import ticks_us, ticks_diff   # Use to get dt value in update()

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chA, chB_pin, chB, direction=1):
        '''Initializes an Encoder object.
        direction: 1 = counter increase -> position increase; -1 = flip (use if position goes negative when you expect positive).
        '''
        self.Tim = Timer(tim, period=0xFFFF, prescaler=0)
        self.Tim.channel(chA, pin=chA_pin, mode=Timer.ENC_AB)
        self.Tim.channel(chB, pin=chB_pin, mode=Timer.ENC_AB)
        self.direction = direction

        self.position = 0
        self.prev_count = 0
        self.delta = 0
        self.dt = 0
        self.last = ticks_us()
    
    def update(self):
        '''Delta = change in timer counter only (16-bit wrap). position += delta.'''
        curr = self.Tim.counter()
        self.delta = curr - self.prev_count
        if self.delta > 32767:
            self.delta -= 65536
        elif self.delta < -32768:
            self.delta += 65536
        self.position += self.direction * self.delta
        self.prev_count = curr
        self.dt = ticks_diff(ticks_us(), self.last)
        self.last = ticks_us()
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return (self.delta*self.direction)/self.dt
    def get_rpm(self):
        '''Returns the most recently updated value of velocity as determined
           within the update() method'''
        return self.get_velocity()*60*1000000/1444
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero.'''
        self.Tim.counter(0)
        self.position = 0
        self.prev_count = 0