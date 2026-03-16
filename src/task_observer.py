# -*- coding: utf-8 -*-
"""
Created on Thu Mar  5 15:25:02 2026

@author: maxwe
"""
from task_share   import Share
import micropython
from IMU_driver import BNO055
from ulab import numpy as np
from pyb import I2C
#from ulab import math as math
i2c = I2C(3, I2C.CONTROLLER, baudrate=100000)
IMU = BNO055(i2c,0x28)

S0_WAIT  = micropython.const(0)  # State 0 wait for command
S1_RUN   = micropython.const(1)  # State 1 run observer

class task_observer:
    
    def __init__(self,IMU,R_pos, R_err,L_pos, L_err, estx, esty, start, current_x, current_y, heading):

        '''
        Initializes the task_observer class. Task observer is responsible for estimating the state of the robot.

        :param: IMU (The IMU object)
        :param: R_pos (The right position for the observer)
        :param: R_err (The right error for the observer)
        :param: L_pos (The left position for the observer)
        :param: L_err (The left error for the observer)
        :param: estx (The estimated x position for the observer)
        :param: esty (The estimated y position for the observer)
        :param: start (The start flag for the observer)
        :param: current_x (The current x position for the observer)
        :param: current_y (The current y position for the observer)
        :param: heading (The heading for the observer)

        notes: 
        '''
        self.state = 0
        
        #values for motor input voltages taken from controller
        #values must be multiplied by 4.5(or read voltage) to convert from 
        #effort to voltage
        
        self.R_vol = R_err  #Share
        self.L_vol = L_err  #Share 
        
        
        self.R_pos = R_pos
        self.L_pos = L_pos
        
        self.IMU = IMU    #IMU object
        
        self.x   = np.array([0,0,0,0]) #temporary
        
        self.A_D = np.array([[0.7427	,0.0000	,0.2494	,0.2494],
                        [0,-0.0061,0,0],
                        [-0.1212,0,0.3192,0.3095],
                        [-0.1212,0,0.3095,0.3192]])
        
        self.B_D = np.array([[0.1962, 0.1962, 0.1287	,0.1287	,0.0000, 0.0000],
                        [0.0000, 0.0000, -0.0071,0.0071,0.0001,	0.0039],
                        [0.7137, 0.4156, 0.0606, 0.0606, 0.0000,-1.8098],
                        [0.4156, 0.7137, 0.0606, 0.0606, 0.0000, 1.8098]])
        
        self.C_D = np.array([[1.0,-70.0,0.0,0.0],
                             [1.0,70.0,0.0,0.0],
                             [0.0,1.0,0.0,0.0],
                             [0.0,0.0,-0.25,.025]])
    
        #self.Y_vector = States #Queue of estimated state vectors
        self.estx = estx #Queue for storing estimaed position
        self.esty = esty
        self.current_x = current_x
        self.current_y = current_y
        self.heading = heading
        
        self.start = start #Share for starting
    
    def run(self):
        '''
        Runs the task_observer class as a Finite State Machine with states with only two states, wait and run.
        :param: None
        :returns: None
        :notes: calculates the state of the robot based on the motor efforts and the IMU data.
        '''
        while True:
#==============================================================================
            if self.state == S0_WAIT:
                '''
                State 0: Wait
                The robot is in wait and waiting for the start flag to be true.
                If the start flag is true, the robot will transition to state 1, run.
                '''
                if self.start.get():
                    self.IMU.set_mode("NDOF")
                    init_sr = (149/2)*self.IMU.heading()
                    init_sl = -(149/2)*self.IMU.heading()
                    self.offset_sr = init_sr - self.R_pos.get()
                    self.offset_sl = init_sl - self.L_pos.get()
                    self.state = S1_RUN
#==============================================================================        
            elif self.state == S1_RUN:
                '''
                State 1: Run
                The robot is in run and calculating the state of the robot based on the motor efforts and the IMU data.
                The robot will then store the state of the robot in the estimated x and y position queues.
                The robot will then store the state of the robot in the current x and y position queues.
                The robot will then store the state of the robot in the heading queue.
                '''
                #grabs the motor efforts and scales to 4.5V
                vr = self.R_vol.get() * 4.5
                vl = self.L_vol.get() * 4.5
                #grabs encoder position and scales it to mm
                sr = (self.R_pos.get()+self.offset_sr) / 633345
                sl = (self.L_pos.get()+self.offset_sl) / 633345
                heading = self.IMU.heading()
                yaw = self.IMU.yaw_rate()
                
                # y = np.array([sl,sr,heading,yaw])
                # u = np.array([vl,vr])
                
                u_star = np.array([vl,vr,sl,sr,heading,yaw])
                
                #matrix operations
                x_next = np.dot(self.A_D,self.x) + np.dot(self.B_D,u_star)
                y_est = np.dot(self.C_D,self.x)
                
                #store these values now somewhere
                #finally
                self.x = x_next
                
                self.estx.put(self.x[0])
                self.esty.put(self.x[1])
                
                self.current_x.put(self.x[0])
                self.current_y.put(self.x[1])
                
                self.heading.put(IMU.heading())
                
                if self.estx.full():
                    self.start.put(False)
                    self.state = S0_WAIT
#==============================================================================
            yield self.state