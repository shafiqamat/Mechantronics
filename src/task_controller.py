# -*- coding: utf-8 -*-
"""
Created on Thu Feb  5 13:24:14 2026

@author: maxwe
"""
from task_share   import Share
from pyb import Pin
import micropython
import time
import linesen


S0_IDLE  = micropython.const(0)  # State 0 wait for turn
S1_STEP  = micropython.const(1)  # State 1 positional setpoint controll
S2_LINE  = micropython.const(2)  # State 2 line following controller
S3_state_estimation = micropython.const(3)  # State 3 state estimation
S4_turn = micropython.const(4)  # State 4 turn


class task_controller:
    
    def __init__(self, Kp, Ki, setpoint,speed, start, line, R_pos, R_err, L_pos, L_err,
                 centroidValues, centroidTime, estx, esty, current_x, current_y, heading):
        '''
        Initializes the task_controller class. Task controller is responsible for how the robot/motors respond to the line sensor inputs.

        :param: Kp (The proportional gain for the line following controller)
        :param: Ki (The integral gain for the line following controller)
        :param: setpoint (The setpoint for the line following controller)
        :param: speed (The speed for the line following controller)
        :param: start (The start flag for the line following controller)
        :param: line (The line flag for the line following controller)
        :param: R_pos (The right position for the line following controller)
        :param: R_err (The right error for the line following controller)
        :param: L_pos (The left position for the line following controller)
        :param: L_err (The left error for the line following controller)
        :param: centroidValues (The centroid values for the line following controller)
        :param: centroidTime (The centroid time for the line following controller)
        :param: estx (The estimated x position for the line following controller)
        :param: esty (The estimated y position for the line following controller)
        :param: current_x (The current x position for the line following controller)
        :param: current_y (The current y position for the line following controller)
        :param: heading (The heading for the line following controller)
        '''
        #global values
        #--------------------------------       
        self.Kp = Kp
        self.Ki = Ki
        self._Kpleft = 0.0
        self._Kpright = 0.0
        self._Kileft = 0.0
        self._Kiright = 0.0
        self.setpoint = setpoint
        self.speed = speed
        self._base_speed = 30
        self.start = start
        self.line = line
        #motor specific values
        #--------------------------------
        self.R_pos = R_pos
        self.R_err = R_err
        self.L_pos = L_pos
        self.L_err = L_err
        #queues for line following
        #--------------------------------
        self.centroids = centroidValues
        self.times = centroidTime 
        #things needed for calculation (not shared)
        #--------------------------------
        self.ILerr = 0.0
        self.IRerr = 0.0
        self.totErr = 0.0
        self.state = 0
        self.LineError = 0
        self.c_count = 0
        self.estx = estx
        self.esty = esty
        self.current_x = current_x
        self.current_y = current_y
        self._turn = 0
        self.init_turn_flag = False
        self.heading = heading

        self.sens = linesen.sensor(Pin.cpu.A6,Pin.cpu.A7,
                                  Pin.cpu.B6,
                                  Pin.cpu.C7,Pin.cpu.A9)

    def run(self):
        '''
        Runs the task_controller class as a Finite State Machine with states for idle, step, line, state estimation, and turn.
        :param: None
        :returns: None
        '''
        while True:
#==============================================================================
            if self.state == S0_IDLE:
                '''
                State 0: Idle
                The robot is idle and waiting for the start flag to be true and the line flag to be true.
                If the start flag is true and the line flag is true, the robot will transition to state 2, line following.
                If the start flag is true and the line flag is false, the robot will transition to state 1, positional setpoint control.
                '''
                if self.start.get() and self.line.get():
                    self.state = S2_LINE
                    self.c_count = 0
                elif self.start.get():
                    self.state = S1_STEP
#==============================================================================
            elif self.state == S1_STEP:
                '''
                State 1: Positional Setpoint Control
                The robot is in positional setpoint control and waiting for the start flag to be false.
                If the start flag is false, the robot will transition to state 0, idle.
                If the start flag is true, the robot will continue to control the motors to the setpoint.
                The robot will calculate the error between the setpoint and the current position of the motors.
                The robot will then calculate the proportional and integral terms of the error.
                The robot will then calculate the effort for the left and right motors.
                The robot will then set the effort for the left and right motors.
                The robot will then put the effort for the left and right motors into the left and right error queues.
                '''
                if not self.start.get():
                    self.state = S0_IDLE
                    self.ILerr = 0.0
                    self.IRerr = 0.0
                    continue

                Lerror = self.setpoint.get() - self.L_pos.get()
                Rerror = self.setpoint.get() - self.R_pos.get()

                Kp = self.Kp.get()
                Ki = self.Ki.get()

                PLerr = Kp * Lerror
                PRerr = Kp * Rerror
                # Integral with dt so Ki has consistent units
                self.ILerr += Lerror * 0.05
                self.IRerr += Rerror * 0.05

                raw_L = PLerr + self.ILerr * Ki
                raw_R = PRerr + self.IRerr * Ki
                effort_L = max(-100, min(100, raw_L))
                effort_R = max(-100, min(100, raw_R))

                if raw_L != effort_L:
                    self.ILerr -= Lerror * 0.05
                if raw_R != effort_R:
                    self.IRerr -= Rerror * 0.05

                self.L_err.put(effort_L)
                self.R_err.put(effort_R)
#==============================================================================
            elif self.state == S2_LINE:
                '''
                State 2: Line Following
                The robot is in line following and waiting for the start flag to be false.
                If the start flag is false, the robot will transition to state 0, idle.
                If the start flag is true, the robot will continue to control the motors to the line sensor inputs.
                The robot will calculate the error between the line sensor inputs and the setpoint.
                The robot will then calculate the proportional and integral terms of the error.
                The robot will then calculate the effort for the left and right motors.
                The robot will then set the effort for the left and right motors.
                The robot will then put the effort for the left and right motors into the left and right error queues.
                The robot will then transition to state 3, state estimation, if the robot is within the waypoint range (determined from encoders).
                The robot will then transition to state 0, idle, if the start flag is false.
                '''
                
                #get values from elsewhere 
                if self.c_count == 0:
                    self._start_time = time.ticks_ms()
                    self.c_count = 1

                if self.sens.get_pos() != 100:
                    self.LineError = self.sens.get_pos()
                error = self.LineError

                # add error to centroid queue and time to time queue
                # time_elapsed = time.ticks_diff(time.ticks_ms(), self._start_time)
                # self.centroids.put(error)
                # self.times.put(time_elapsed)

                Kp = self.Kp.get()
                Ki = self.Ki.get()
                
                #set base speeds
                leff = self.speed.get()
                reff = self.speed.get()
                #accumulate error 
                self.totErr += error * 0.05
                #do preportional controll
                error = error * Kp
                #do intergral control
                error += self.totErr * Ki
                #finally set effort
                leff += error  
                reff -= error
                self.L_err.put(leff)
                self.R_err.put(reff)

                if self.current_x.get() in range(100,150) and self.current_y.get() in range(100,150):
                    self.line.put(False)
                    self.state = S3_state_estimation
                
                if self.start.get() == False:
                    self.L_err.put(0)
                    self.R_err.put(0)
                    self.totErr = 0.0
                    self.state = S0_IDLE
#==============================================================================
            elif self.state == S3_state_estimation:
                '''
                State 3: State Estimation
                The robot is in state estimation and waiting for the start flag to be false.
                If the start flag is false, the robot will transition to state 0, idle.
                If the start flag is true, the robot will continue to estimate the state of the robot.
                The robot will calculate the error between the estimated x and y position and the waypoint.
                The robot will then calculate the proportional and integral terms of the error.
                The robot will then calculate the effort for the left and right motors.
                The robot will then set the effort for the left and right motors.
                The robot will then put the effort for the left and right motors into the left and right error queues.
                The robot will then transition to state 4, turn, if the robot is within the turn range.
                The robot will then transition to state 0, idle, if the start flag is false.
                '''
                way_point_1 = 150
                leff = self._base_speed
                reff = self._base_speed
                x_error = way_point_1 - self.current_x.get()
                acc_error_L = x_error * 0.05
                acc_error_R = x_error * 0.05
                self._Kpleft = self._Kpleft * x_error
                self._Kpright = self._Kpright * x_error
                self._Kileft = self._Kileft * acc_error_L
                self._Kiright = self._Kiright * acc_error_R
                leff += self._Kpleft + self._Kileft
                reff += self._Kpright + self._Kiright
                self.L_err.put(leff)
                self.R_err.put(reff)
                if self.current_x.get() in range(way_point_1-10,way_point_1+10):
                    self.state = S4_turn
                    self._turn = 90
                    self.init_turn_flag = True

#==============================================================================
            elif self.state == S4_turn:
                '''
                State 4: Turn
                The robot is in turn and waiting until the turn is complete.
                The robot will then transition to state 3, state estimation, once turn is complete
                '''
                if self.init_turn_flag:
                    old_heading = self.heading.get()
                    self.init_turn_flag = False
                if self._turn == 90:
                    self.L_err.put(25*0.045)
                    self.R_err.put(-25)
                    if self.heading.get() - old_heading >= 90:
                        self.state = S3_state_estimation
                elif self._turn == -90:
                    self.L_err.put(-25*0.045)
                    self.R_err.put(25)
                    if self.heading.get() - old_heading <= -90:
                        self.state = S3_state_estimation
                elif self._turn == 180:
                    self.L_err.put(-25*0.045)
                    self.R_err.put(25)
                    if self.heading.get() - old_heading >= 180:
                        self.state = S3_state_estimation
              
#==============================================================================
            yield self.state
                