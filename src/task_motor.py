''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
from motor        import Motor
from encoder      import Encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control
S3_LINE = micropython.const(3) # State 3 - run line following 
S4_TURN = micropython.const(4) # State 4 - turn


class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: Motor, enc: Encoder, start, startTimeRef, line,
                 pos, err, go, dataValues, timeValues):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: Motor = mot        # A motor object
        
        self._enc: Encoder      = enc        # An encoder object
        
        self._goFlag: Share     = go    # A share object representing a
                                             # flag to start data collection
                                            
        self._err    = err   

        self._pos    = pos     

        self._start = start
        self._startTimeRef = startTimeRef  # Share: collection start (us), same for both motors
        self._dataValues = dataValues
        self._timeValues = timeValues
        
        self._line = line

        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
#==============================================================================            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                # print("Initializing motor task")
                self._mot.enable()
                self._state = S1_WAIT
#==============================================================================              
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._line.get() & self._goFlag.get():
                    self._start.put(True)
                    self._state = S3_LINE                    
            
                elif self._goFlag.get():
                    self._start.put(True)
                    
                    # print("Starting motor loop")
                    
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._enc.zero()
                    self._startTime = ticks_us()
                    self._state = S2_RUN
#==============================================================================               
            elif self._state == S2_RUN:  # Closed-loop control state
                self._enc.update()
                pos = self._enc.get_position()
                self._pos.put(pos)

                t = ticks_us()
                elapsed_us = ticks_diff(t, self._startTimeRef.get())
                if elapsed_us < 0:
                    elapsed_us = 0

                driver = self._err.get()
                self._mot.set_effort(driver)

                # Store POSITION only (not effort); time in us from shared start
                self._dataValues.put(pos)
                self._timeValues.put(elapsed_us)
                
                # When the queues are full, data collection is over
                if self._dataValues.full():
                    # print("Exiting motor loop")
                    self._state = S1_WAIT
                    self._goFlag.put(False)
                    self._start.put(False)
                    self._mot.set_effort(0)
#==============================================================================
            elif self._state == S3_LINE:
                
                self._enc.update()
                self._mot.set_effort(self._err.get())
                self._pos.put(self._enc.get_position())
                
                if self._goFlag.get() == False:
                    self._start.put(False)
                    self._mot.set_effort(0)
                    self._state = S1_WAIT
                    
            elif self._state == S4_TURN:
                self._mot.set_effort(self._err.get())

#==============================================================================
                
            yield self._state