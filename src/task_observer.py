"""State observer for robot pose estimation using encoders and the IMU."""
from task_share   import Share
import micropython
from IMU_driver import BNO055
from ulab import numpy as np
import math as math


S0_WAIT  = micropython.const(0)  # State 0 wait for command
S1_RUN   = micropython.const(1)  # State 1 run observer

class task_observer:
    """Observer task that estimates robot position and heading."""

    def __init__(self,IMU,R_pos, R_err,L_pos, L_err, estx, esty, start, current_x, current_y,heading,dist,accl):
        """Initialize the observer task.

        Args:
            IMU: BNO055 driver instance.
            R_pos: Share containing the right encoder position.
            R_err: Share containing the right motor effort command.
            L_pos: Share containing the left encoder position.
            L_err: Share containing the left motor effort command.
            estx: Queue reserved for estimated x-history samples.
            esty: Queue reserved for estimated y-history samples.
            start: Share used to enable or stop the observer.
            current_x: Share containing the latest estimated x position.
            current_y: Share containing the latest estimated y position.
            heading: Share containing the latest estimated heading.
            dist: Share containing cumulative estimated travel distance.
            accl: Share containing the latest acceleration sample.
        """
        self.state = 0
        
        #values for motor input voltages taken from controller
        #values must be multiplied by 4.5(or read voltage) to convert from 
        #effort to voltage
        
        self.R_vol = R_err  #Share
        self.L_vol = L_err  #Share 
        
        
        self.R_pos = R_pos
        self.L_pos = L_pos
        
        self.IMU = IMU    #IMU object
        
        self.heading = heading
        
        self.x   = np.array([0,0,0,0]) #temporary
        
        self.A_D = np.array([[0.7427	,0.0000	,0.2494	,0.2494],
                        [0,0.0061,0,0],
                        [-0.1212,0,0.3192,0.3095],
                        [-0.1212,0,0.3095,0.3192]])
        
        self.B_D = np.array([[0.1962, 0.1962, 0.1287	,0.1287	,0.0000, 0.0000],
                        [0.0000, 0.0000, -0.0071,0.0071,0.0001,	0.0039],
                        [0.7137, 0.4156, 0.0606, 0.0606, 0.0000,-1.8098],
                        [0.4156, 0.7137, 0.0606, 0.0606, 0.0000, 1.8098]])
        
        self.C_D = np.array([[1.0,-70.0,0.0,0.0],
                             [1.0,70.0,0.0,0.0],
                             [0.0,1.0,0.0,0.0],
                             [0.0,0.0,-0.25,0.25]])
    
        #self.Y_vector = States #Queue of estimated state vectors
        self.estx = estx #Queue for storing estimaed position
        self.esty = esty
        self.current_x = current_x
        self.current_y = current_y
        self.dist = dist
        self.accl = accl
        
        self.start = start #Share for starting
    
    def run(self):
        """Run one iteration of the observer state machine.

        Yields:
            int: The current observer state identifier.
        """
        while True:
#==============================================================================
            if self.state == S0_WAIT:
                if self.start.get():
                    self.dist.put(0)
                    self.current_x.put(0)
                    self.current_y.put(0)
                    self.IMU.set_mode("NDOF")
                    #zero heading
                    self.head_off = self.IMU.heading()
                    init_sr = (149/2)*self.IMU.heading()
                    init_sl = -(149/2)*self.IMU.heading()
                    self.offset_sr = init_sr - self.R_pos.get()
                    self.offset_sl = init_sl - self.L_pos.get()
                    self.state = S1_RUN
#==============================================================================        
            elif self.state == S1_RUN:
                #grabs the motor efforts and scales to 4.5V
                vr = self.R_vol.get() * 4.5/100
                vl = self.L_vol.get() * 4.5/100
                #saturation
                if vr > 4.5:
                    vr = 4.5
                elif vr < -4.5:
                    vr = -4.5
                if vl > 4.5:
                    vl = 4.5
                elif vl < -4.5:
                    vl = -4.5
                # #grabs encoder position and scales it to mm
                sr = (self.R_pos.get()+self.offset_sr) / 633345
                sl = (self.L_pos.get()+self.offset_sl) / 633345
                # heading = self.IMU.heading()
                yaw = self.IMU.yaw_rate()
                
                # y = np.array([sl,sr,heading,yaw])
                # u = np.array([vl,vr])
                heading = self.IMU.heading() - self.head_off
                if heading > 360:
                    heading -= 360
                elif heading < 0:
                    heading += 360
                
                u_star = np.array([vl,vr,sl,sr,heading,yaw])
                
                #matrix operations
                x_next = np.dot(self.A_D,self.x) + np.dot(self.B_D,u_star)
                y_est = np.dot(self.C_D,self.x)
                
                #store these values now somewhere
                #finally
                
                
                # self.estx.put(self.x[0])
                # self.esty.put(self.x[1])
                self.x[0] = self.x[0] / .87
                
                self.current_x.put(self.current_x.get() + self.x[0]*math.cos(math.radians(heading)))
                self.current_y.put(self.current_y.get() + self.x[0]*math.sin(math.radians(heading)))
                self.dist.put(self.dist.get() + self.x[0])
                self.heading.put(heading)
                self.accl.put(self.IMU.get_accel()[0])

                
                self.x = x_next
                if not self.start.get():
                    self.state = S0_WAIT
#==============================================================================
            yield self.state
