"""Serial user-interface task for tuning, test runs, and data logging."""
from pyb import USB_VCP,UART
import time
from task_share import Share, Queue
import cotask
import micropython

S0_MM   = micropython.const(0) # State 0 - main menu
S1_INP  = micropython.const(1) # State 1 - main menu input 
S2_HLP  = micropython.const(2) # State 2 - help state
S3_KP   = micropython.const(3) # State 3 - Kp selection
S4_KI   = micropython.const(4) # State 4 - Ki selection
S5_GO   = micropython.const(5) # State 5 - start data collection
S6_WAIT = micropython.const(6) # State 6 - wait for data to be done
S7_DATA = micropython.const(7) # State 7 - put data into csv
S8_STP  = micropython.const(8) # State 8 - setpoint selection
S9_LINE = micropython.const(9) # State 9 - line following menu
S10_LIN = micropython.const(10)# State 10- line follow input
S11_SETS= micropython.const(11)# State 11- set standard follow speed
S12_HOLD= micropython.const(12)# State 12- here while following line
S13_DATL= micropython.const(13)# State 13- print the data from line follow
S14_track= micropython.const(14)# State 14- track the line



UI_prompt = ">: "

class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    def __init__(self, setpoint, gainP, gainI, speed, line, leftMotorGo, rightMotorGo,
                 leftDataValues, leftTimeValues, rightDataValues, rightTimeValues,
                 centroidValues, centroidTime, estx, esty,
                 startTime=None):
        """Initialize the user-interface task.

        Args:
            setpoint: Share containing the motor-test target position.
            gainP: Share containing the proportional gain.
            gainI: Share containing the integral gain.
            speed: Share containing the default line-following speed.
            line: Share indicating whether line-following mode is active.
            leftMotorGo: Share used to start or stop the left motor task.
            rightMotorGo: Share used to start or stop the right motor task.
            leftDataValues: Queue of logged left-motor position samples.
            leftTimeValues: Queue of timestamps for left-motor samples.
            rightDataValues: Queue of logged right-motor position samples.
            rightTimeValues: Queue of timestamps for right-motor samples.
            centroidValues: Queue reserved for line-following telemetry.
            centroidTime: Queue reserved for line-following timestamps.
            estx: Queue containing estimated x-position samples.
            esty: Queue containing estimated y-position samples.
            startTime: Optional share storing the common logging start time in
                microseconds.
        """
        self._state = S0_MM
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._ser = UART(5,9600)
        #self._ser = USB_VCP()
        self._setpoint = setpoint
        self._gainP = gainP
        self._gainI = gainI
        self.buffer = ""
        self._leftDataValues = leftDataValues
        self._leftTimeValues = leftTimeValues
        self._rightDataValues = rightDataValues
        self._rightTimeValues = rightTimeValues
        self._centroidValues = centroidValues
        self._centroidTime = centroidTime
        self._startTime = startTime
        self._speed = speed
        self._line = line
        
        self.estx = estx
        self.esty = esty
        
        self._ser.write("User Task object instantiated")
        
        self._collectflag: int    = 0
        self._time: float         = time.localtime()
        self._time_format: float  = 0.0
        self._filename: str       = ""
        self._line_follow_filename: str= ""
        self.track_flag = False
        
    def run(self):
        """Run one iteration of the user-interface state machine.

        Yields:
            int: The current UI state identifier.
        """
        
        while True:
#==============================================================================            
            if self._state == S0_MM: # main menu
            #vestigular
                #self._setpoint.put(2000)
                
    
                self._ser.write("+--------------------+\r\n")
                self._ser.write("|Testing Main Menu\r\n")
                self._ser.write("+---+----------------+\r\n")
                self._ser.write("| h | help menu\r\n")
                self._ser.write(f"| k | gain selection, Kp:{self._gainP}, Ki{self._gainI}\r\n")
                self._ser.write(f"| s | setpoint selection, current:{self._setpoint}\r\n")
                self._ser.write("| g | start the test\r\n")
                self._ser.write("| l | line following menu\r\n")
                self._ser.write("| t | task list table\r\n")
                self._ser.write("+---+----------------+\r\n")

                self._ser.write(UI_prompt)
                self._state = S1_INP
#==============================================================================                
            elif self._state == S1_INP: # Wait for UI commands
                # Wait for at least one character in serial buffer
                if self._ser.any():
                    # Read the character and decode it into a string
                    inChar = self._ser.read(1).decode()
                    
                    #checks if the inputed character is one in menu
                    
                    if inChar in {"h", "H"}:
                        self._ser.write("help menu")
                        self._state = S2_HLP
                        
                    elif inChar in {"k", "K"}:
                        self.buffer = ""
                        self._ser.write("select Kp:\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S3_KP
                        
                    elif inChar in {"s","S"}:
                        self.buffer = ""
                        self._ser.write("select setpoint\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S8_STP
                        
                    elif inChar in {'g','G'}:
                        self._ser.write("Waiting for go command: 'l' for left, 'r' for right\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S5_GO
                        
                    elif inChar in {'l','L'}:
                        self._line.put(True)
                        self._state = S9_LINE
                        
                    elif inChar in {'t','T'}:
                        self._ser.write(cotask.task_list.__repr__())
                        self._state = S0_MM
#============================================================================== 
            elif self._state == S2_HLP:
                # waits for user to input a character to exit help menu
                if self._ser.any():
                    self._state = S0_MM
#============================================================================== 
            elif self._state == S3_KP:
                if self._ser.any():
                    #multicharacter input
                    char = self._ser.read(1).decode()
                    #check if digit
                    if char in {"1",'2','3','4','5','6','7','8','9','0'}:
                        self._ser.write(char)
                        self.buffer += char
                    #check if decimal 
                    elif char == '.' and not '.' in self.buffer:
                        self._ser.write(char)
                        self.buffer+= char
                    
                    #check for backspace/rubout
                    elif char == '\x7f' and len(self.buffer) > 0:
                        self._ser.write(char)
                        self.buffer = self.buffer[:-1]
                    
                    #when enter key
                    elif char in {'\r','\n'}:
                        if len(self.buffer) == 0:
                            self._ser.write('value unchanged\r\n')
                            self.buffer = ''
                            self._ser.write("\r\nselect Ki:\r\n")
                            self._state = S4_KI
                        elif self.buffer not in {'-','.'}:
                            self._gainP.put(float(self.buffer))
                            self.buffer = ''
                            self._ser.write("\r\nselect Ki:\r\n")
                            self._ser.write(UI_prompt)
                            self._state = S4_KI
#============================================================================== 
            elif self._state == S4_KI:
                if self._ser.any():
                    #multicharacter input
                    char = self._ser.read(1).decode()
                    #check if digit
                    if char in {"1",'2','3','4','5','6','7','8','9','0'}:
                        self._ser.write(char)
                        self.buffer += char
                    #check if decimal 
                    elif char == '.' and not '.' in self.buffer:
                        self._ser.write(char)
                        self.buffer+= char
                    
                    #check for backspace/rubout
                    elif char == '\x7f' and len(self.buffer) > 0:
                        self._ser.write(char)
                        self.buffer = self.buffer[:-1]
                    
                    
                    #when enter key
                    elif char in {'\r','\n'}:
                        self._ser.write('\r\n')
                        if len(self.buffer) == 0:
                            self._ser.write('value unchanged\r\n')
                            self.buffer = ''
                            self._state = S0_MM
                        elif self.buffer not in {'-','.'}:
                            self._gainI.put(float(self.buffer))
                            self.buffer = ''
                            self._state = S0_MM
#============================================================================== 
            elif self._state == S8_STP:
                if self._ser.any():
                    #multicharacter input
                    char = self._ser.read(1).decode()
                    #check if digit
                    if char in {"1",'2','3','4','5','6','7','8','9','0'}:
                        self._ser.write(char)
                        self.buffer += char
                    #check if decimal 
                    elif char == '.' and not '.' in self.buffer:
                        self._ser.write(char)
                        self.buffer += char
                    #check for minus
                    elif char == "-" and len(self.buffer) == 0:
                        self._ser.write(char)
                        self.buffer += char
                        
                    #check for backspace/rubout
                    elif char == '\x7f' and len(self.buffer) > 0:
                        self._ser.write(char)
                        self.buffer = self.buffer[:-1]
                    
                    #when enter key
                    elif char in {'\r','\n'}:
                        if len(self.buffer) == 0:
                            self._ser.write('value unchanged\r\n')
                            self.buffer = ''
                            self._state = S0_MM
                        elif self.buffer not in {'-','.'}:
                            self._setpoint.put(float(self.buffer))
                            self._state = S0_MM
#==============================================================================
            elif self._state == S5_GO:
                # Wait for at least one character in serial buffer
                if self._ser.any():
                    # Read the character and decode it into a string
                    inChar = self._ser.read(1).decode()
                    # If the character is an upper or lower case "l", start data
                    # collection on the left motor and if it is an "r", start
                    # data collection on the right motor
                    if inChar in {"l", "L"}:
                        self._ser.write(f"{inChar}\r\n")
                        if self._startTime is not None:
                            self._startTime.put(time.ticks_us())
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting left motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._state = S6_WAIT
                        self._collectflag = 1
                    elif inChar in {"r", "R"}:
                        self._ser.write(f"{inChar}\r\n")
                        if self._startTime is not None:
                            self._startTime.put(time.ticks_us())
                        self._rightMotorGo.put(True)
                        self._ser.write("Starting right motor loop...\r\n")
                        self._ser.write("Starting data collection...\r\n")
                        self._ser.write("Please wait... \r\n")
                        self._collectflag = 1
                        self._state = S6_WAIT
#============================================================================== 
            elif self._state == S6_WAIT:
                # While the data is collecting (in the motor task) block out the
                # UI and discard any character entry so that commands don't
                # queue up in the serial buffer
                if self._ser.any(): self._ser.read(1)
                
                # When both go flags are clear, the data collection must have
                # ended and it is time to print the collected data.
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Time, Position\r\n")
                    self._state = S7_DATA
#==============================================================================            
            elif self._state == S7_DATA:
                if self._collectflag == 1:
                    self._time_format = "{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
                        self._time[0], self._time[1], self._time[2],
                        self._time[3], self._time[4], self._time[5])
                    self._filename = "/flash/log_{}.csv".format(self._time_format)
                    with open(self._filename, 'w') as file:
                        file.write("Time_us, Position, Motor\r\n")
                    self._ser.write("Logging to file: {}\r\n".format(self._filename))
                    self._collectflag = 0
                # Drain left then right queues; data is position, not effort
                has_left = self._leftDataValues.any()
                has_right = self._rightDataValues.any()
                if has_left:
                    t = self._leftTimeValues.get()
                    d = self._leftDataValues.get()
                    self._ser.write(f"{t/1000:.0f},{d},\r\n")
                    with open("{}".format(self._filename), 'a') as file:
                        file.write(f"{t/1000},{d},\r\n")
                elif has_right:
                    t = self._rightTimeValues.get()
                    d = self._rightDataValues.get()
                    self._ser.write(f"{t/1000:.0f},{d},\r\n")
                    with open("{}".format(self._filename), 'a') as file:
                        file.write(f"{t/1000},{d},\r\n")
                else:
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Waiting for go command: 'l' for left, 'r' for right\r\n")
                    self._ser.write(UI_prompt)
                    self._filename = ""
                    self._state = S0_MM
#==============================================================================
            elif self._state == S9_LINE:
                self._ser.write("+--------------------+\r\n")
                self._ser.write("|Line Following menu\r\n")
                self._ser.write("+---+----------------+\r\n")
                self._ser.write(f"| s | speed selection: current {self._speed}\r\n")
                self._ser.write("| r | return to main menu\r\n")
                self._ser.write("| t | start the test\r\n")
                self._ser.write("| g | gametrack\r\n")
                self._ser.write("+---+----------------+\r\n")
                
                self._ser.write(UI_prompt)
                self._state = S10_LIN
#==============================================================================
            elif self._state == S10_LIN:
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    
                    if inChar in {'s','S'}:
                        self._ser.write("select line follow steady speed\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S11_SETS
                        
                    elif inChar in {'r','R'}:
                        self._line.put(False)
                        self._state = S0_MM
                        
                    elif inChar in {'t','T'}:
                        self._ser.write('Press any to stop\r\n')
                        if self._startTime is not None:
                            self._startTime.put(time.ticks_us())
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._state = S12_HOLD
                        
                    elif inChar in {'g','G'}:
                        self._state = S14_track
#==============================================================================
            elif self._state == S11_SETS:
                if self._ser.any():
                    #multicharacter input
                    char = self._ser.read(1).decode()
                    #check if digit
                    if char in {"1",'2','3','4','5','6','7','8','9','0'}:
                        self._ser.write(char)
                        self.buffer += char
                    
                    #check for backspace/rubout
                    elif char == '\x7f' and len(self.buffer) > 0:
                        self._ser.write(char)
                        self.buffer = self.buffer[:-1]
                    
                    #when enter key
                    elif char in {'\r','\n'}:
                        self._ser.write('\r\n')
                        if len(self.buffer) == 0:
                            self._ser.write('value unchanged\r\n')
                            self.buffer = ''
                            self._state = S9_LINE
                        elif float(self.buffer) > 100.0:
                            self._ser.write('no value above 100 allowed\r\n')
                            self.buffer = ''
                            self._state = S9_LINE
                        else:
                            self._speed.put(float(self.buffer))
                            self.buffer = ''
                            self._state = S9_LINE
#==============================================================================
            elif self._state == S12_HOLD:
                if self._ser.any():
                    self._ser.read()
                    self._rightMotorGo.put(False)
                    self._leftMotorGo.put(False)
                    if not self.track_flag:
                        self._collectflag = 1
                        self._ser.write("Starting line follow data collection...\r\n")
                        self._state = S13_DATL
                    else:
                        self.track_flag = False
                        self._state = S0_MM
#==============================================================================
            elif self._state == S13_DATL:
                if self._collectflag == 1:
                    self._time_format = "{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}".format(
                        self._time[0], self._time[1], self._time[2],
                        self._time[3], self._time[4], self._time[5])
                    self._line_follow_filename = "/flash/estimated_pos_{}.csv".format(self._time_format)
                    with open(self._line_follow_filename, 'w') as file:
                        file.write("Time_us, Centroid\r\n")
                    self._ser.write("Logging to file: {}\r\n".format(self._line_follow_filename))
                    self._collectflag = 0
                # Drain left then right queues; data is position, not effort
                has_val = self.esty.any()
                has_t = self.estx.any()
                if has_val and has_t:
                    ct = self.estx.get()
                    cv = self.esty.get()
                    self._ser.write(f"{ct:.0f},{cv}\r\n")
                    with open("{}".format(self._line_follow_filename), 'a') as file:
                        file.write(f"{ct:.0f},{cv},\r\n")
                else:
                    self._line_follow_filename = ""
                    self._state = S9_LINE
#==============================================================================
            elif self._state == S14_track:
                self._ser.write('Press any to stop\r\n')
                if self._startTime is not None:
                    self._startTime.put(time.ticks_us())
                self._leftMotorGo.put(True)
                self._rightMotorGo.put(True)
                self.track_flag = True
                self._state = S12_HOLD
                
#==============================================================================
            yield self._state
