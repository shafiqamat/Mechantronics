"""Closed-loop controller and game-state logic for the Romi robot.

This task contains the position controller used for single-motor testing, the
line-following controller, and the state-estimation/game-track logic used
during the final autonomous run.
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
S4_POINT = micropython.const(4) # State 4 point to heading
S5_DIST  = micropython.const(5) # State 5 straight line distance
S6_HUB   = micropython.const(6) # State 6 keeps track of the gamestate
S7_FEEL  = micropython.const(7) # State 7 drive until bump is detected





class task_controller:
    """Controller task for setpoint, line-following, and track navigation."""

    def __init__(self, Kp, Ki, setpoint,speed, start, line, R_pos, R_err, L_pos, L_err,
                 centroidValues, centroidTime, current_x, current_y,heading,dist,accl):
        """Initialize the controller task and cache all shares and queues.

        Args:
            Kp: Share containing the proportional gain.
            Ki: Share containing the integral gain.
            setpoint: Share containing the position target for motor tests.
            speed: Share containing the base line-following speed.
            start: Share used to start or stop the controller flow.
            line: Share indicating whether line-following mode is active.
            R_pos: Share containing the right encoder position.
            R_err: Share used to publish the right motor command.
            L_pos: Share containing the left encoder position.
            L_err: Share used to publish the left motor command.
            centroidValues: Queue reserved for line-following telemetry.
            centroidTime: Queue reserved for line-following timestamps.
            current_x: Share containing the estimated x position.
            current_y: Share containing the estimated y position.
            heading: Share containing the estimated heading.
            dist: Share containing the estimated travel distance.
            accl: Share containing acceleration data from the IMU.
        """
        #global values
        #--------------------------------       
        self.Kp = Kp
        self.Ki = Ki
        self.setpoint = setpoint
        self.speed = speed
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
        self.lastErr = 0.0
        self.state = 0
        self.prev_state = -1
        self.LineError = 0
        self.c_count = 0
        self.offr = 0
        self.offl = 0
        self.current_x = current_x
        self.current_y = current_y
        self.heading = heading
        self.dist = dist
        self.accl = accl
        self.count = 0
        #track data
        #--------------------------------
        self.tracker = 0  #holds the game state
        self.wp = [0,0]
        self.Wthresh = 10
        self.point = 0
        self.trav = 0
        self.acclT = 220
        self.Pthresh = 1.7
        
        
        

        
        self.sens = linesen.sensor(Pin.cpu.A6,Pin.cpu.A7,
                                  Pin.cpu.B6,
                                  Pin.cpu.C7,Pin.cpu.A9)
        
    def run(self):
        """Run one iteration of the controller state machine.

        Yields:
            int: The current controller state identifier.
        """
        while True:
            # When entering a new closed-loop state, reset integral/counters
            # so old accumulated error doesn't bias the new behavior.
            if self.state != self.prev_state:
                if self.state == S4_POINT or self.state == S5_DIST:
                    self.totErr = 0.0
                    self.count = 0
                    self.lastErr = 0.0
                self.prev_state = self.state
#==============================================================================
            if self.state == S0_IDLE:
                if self.start.get() and self.line.get():
                    self.state = S6_HUB
                    self.c_count = 0
                elif self.start.get():
                    self.state = S1_STEP
#==============================================================================
            elif self.state == S1_STEP:
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
                
                #get values from elsewhere 
                if self.c_count == 0:
                    self._start_time = time.ticks_ms()
                    self.c_count = 1

                if self.tracker == 0:
                    error = self.sens.get_pos()
                    if error == 100:
                        error = 0
                else:
                    if self.sens.get_pos() != 100:
                        self.lineError = self.sens.get_pos()
                    error = self.lineError

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
                
                print(self.current_x.get())
                print(self.current_y.get())

                #print(self.heading.get())
                

                if self.current_x.get() > 1350:
                    self.speed.put(12)
                if self.tracker == 7:
                    if (self.wp[0]+self.Wthresh > self.current_x.get() > self.wp[0]-self.Wthresh and 
                    self.wp[1]+self.Wthresh > self.current_y.get() > self.wp[1]-self.Wthresh):
                        self.L_err.put(0)
                        self.R_err.put(0)
                        self.tracker += 1
                        self.state = S6_HUB

                else:
                    if (self.wp[1]+self.Wthresh > self.current_y.get() > self.wp[1]-self.Wthresh):
                        self.L_err.put(0)
                        self.R_err.put(0)
                        self.tracker += 1
                        self.state = S6_HUB
                    
                
                # if self.start.get() == False:
                #     self.L_err.put(0)
                #     self.R_err.put(0)
                #     self.totErr = 0.0
                #     self.state = S0_IDLE                   
#==============================================================================
            elif self.state == S4_POINT:
                #Go to this state when you set point and want to point to an angle
                
                error = (self.heading.get() - self.point)
                if (0 < self.point < 2 or 358<self.point<360) and abs(error) > 180:
                    error = error-360
                
                if abs(error) > 180:
                    error = -error
                    
                
                
                Perr = error * .5 #KP
                clamp = 60
                if Perr > clamp:
                    Perr = clamp
                elif Perr < -clamp:
                    Perr = -clamp
                self.totErr += error *0.05
                windup = 40
                if self.totErr > windup:
                    self.totErr = windup
                elif self.totErr < -windup:
                    self.totErr = -windup
                Ierr = self.totErr * .6 #KI
                
                Derr = (error - self.lastErr) * .2
                
                eff = Perr + Ierr - Derr
                self.lastErr = error
                
                self.L_err.put(-eff*0.4)
                self.R_err.put(eff*0.45)
                
                print (self.heading.get())
                #check if within acceptable range                    
                if abs(error) < self.Pthresh:
                    self.count += 1
                #exit when acceptable for acceptable time
                if self.count > 10: #placeholder threshold
                    self.count = 0
                    self.tracker += 1
                    self.state = S6_HUB
#==============================================================================
            elif self.state == S5_DIST:
                error = (self.dist.get() - self.trav)
                
                Perr = error * 1 # KP
                #clamp P error
                clamp = 20
                if Perr > clamp:
                    Perr = clamp
                elif Perr < -clamp:
                    Perr = -clamp
                    
                self.totErr += error *0.05
                #reduce windup
                windup = 80
                if self.totErr > windup:
                    self.totErr = windup
                elif self.totErr < -windup:
                    self.totErr = -windup
                Ierr = self.totErr * .3 #KI
                
                Derr = (error - self.lastErr) * .1 #KD
                
                
                eff = Perr + Ierr - Derr
                self.lastErr = error
                
                #correct for inconsistent motors
                diff = ((self.L_pos.get()-self.offl) - (self.R_pos.get()-self.offr)) * .1
                
                self.L_err.put(-eff*0.35-diff)
                self.R_err.put(-eff*0.35+diff)
                
                print(self.dist.get())
                # #check if within acceptable range
                if abs(error) < 10:
                    self.count += 1
                #exit when acceptable for acceptable time
                if self.count > 20: #placeholder threshold
                    self.count = 0
                    self.tracker += 1
                    self.state = S6_HUB                
#==============================================================================
            elif self.state == S6_HUB:
                self.offr = self.R_pos.get()
                self.offl = self.L_pos.get()
                if self.tracker == 0: # line follow 
                    self.Wthresh = 5
                    self.wp = [1780, 410]
                    self.state = S2_LINE
                elif self.tracker == 1: #turn in garage
                    self.state = S4_POINT
                    self.point = 0.5
                elif self.tracker == 2: # drive until bump
                    self.state = S7_FEEL
                elif self.tracker == 3: #bump reset
                    self.trav = self.dist.get() + 50
                    self.state = S5_DIST
                elif self.tracker == 4: #turn out of garage
                    self.Pthresh = 2
                    self.point = 90
                    self.state = S4_POINT
                elif self.tracker == 5: #line follow out
                    self.wp = [0, 950]
                    self.state = S2_LINE
                elif self.tracker == 6: # turn once more 
                    self.Pthresh = 2
                    self.point = 180
                    self.state = S4_POINT
                elif self.tracker == 7: #big line follow 
                    self.Wthresh = 20
                    self.state = S2_LINE
                    self.wp = [260,400]
                elif self.tracker == 8: #point to end
                    self.Pthresh = 1.7
                    self.point = 45
                    self.state = S4_POINT
                elif self.tracker == 9: #go to end
                    self.trav = self.dist.get() - 550
                    self.state = S5_DIST
                else:
                    self.state = S0_IDLE
                    self.start.put(False)
#==============================================================================                                 
            elif self.state == S7_FEEL:
                diff = ((self.L_pos.get()-self.offl) - (self.R_pos.get()-self.offr)) * .1

                self.L_err.put(-30-diff)
                self.R_err.put(-30+diff)
                #print(self.accl.get())
                if   abs(self.accl.get()) > self.acclT:
                    self.L_err.put(0)
                    self.R_err.put(0)
                    #-------------------
                    self.tracker += 1
                    self.state = S6_HUB
#==============================================================================
            yield self.state
                
