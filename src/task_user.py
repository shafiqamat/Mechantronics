''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
from pyb import USB_VCP
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
        '''
        Initializes the task_user class. Task user is responsible for reading user input over a serial port, parsing the input for single-character commands,
        and then manipulating shared variables to communicate with other tasks based on the user commands.
        :param: setpoint (The setpoint for the task user)
        :param: gainP (The proportional gain for the task user)
        :param: gainI (The integral gain for the task user)
        :param: speed (The speed for the task user)
        :param: line (The line flag for the task user)
        :param: leftMotorGo (The left motor go flag for the task user)
        :param: rightMotorGo (The right motor go flag for the task user)
        :param: leftDataValues (The left data values for the task user)
        :param: leftTimeValues (The left time values for the task user)
        :param: rightDataValues (The right data values for the task user)
        :param: rightTimeValues (The right time values for the task user)
        :param: centroidValues (The centroid values for the task user)
        :param: centroidTime (The centroid time for the task user)
        :param: estx (The estimated x position for the task user)
        :param: esty (The estimated y position for the task user)
        :param: startTime (The start time for the task user)
        '''
        self._state = S0_MM
        self._leftMotorGo = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._ser = USB_VCP()
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
        
    def run(self):
        '''
        implements the ui as a finite state machine with states for main menu, input, help menu, gain selection, setpoint selection, start the test, line following menu, task list table, and track the line.
        :param: None
        :returns: None
        :notes: The task_user class is a Finite State Machine with states for main menu, input, help menu, gain selection, setpoint selection, start the test, line following menu, task list table, and track the line.
        The task_user class is responsible for reading user input over a serial port, parsing the input for single-character commands,
        and then manipulating shared variables to communicate with other tasks based on the user commands.
        '''
        
        while True:
#==============================================================================            
            if self._state == S0_MM: # main menu
                '''
                State 0: Main Menu
                The robot is in the main menu, this task just displays the main menu before moving to the input state.
                '''
                
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
                self._ser.write("| ft | track the line\r\n")
                self._ser.write("+---+----------------+\r\n")

                self._ser.write(UI_prompt)
                self._state = S1_INP
#==============================================================================                
            elif self._state == S1_INP: # Wait for UI commands
                '''
                State 1: Input
                The robot is in the input state and waiting for user input.
                The robot will display the input state and wait for user input.
                The robot will then transition to the help menu if the user inputs 'h'.
                The robot will then transition to the gain selection state if the user inputs 'k'.
                The robot will then transition to the setpoint selection state if the user inputs 's'.
                The robot will then transition to the start the test state if the user inputs 'g'.
                The robot will then transition to the line following menu state if the user inputs 'l'.
                The robot will then transition to the task list table state if the user inputs 't'.
                The robot will then transition to the track the line state if the user inputs 'ft'.
                '''
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
                    elif inChar in {'ft','FT'}:
                        self._state = S14_track
#============================================================================== 
            elif self._state == S2_HLP:
                '''
                State 2: Help Menu
                The robot is in the help menu and waiting for user input.
                The robot will display the help menu and wait for user input.
                The robot will then transition to the main menu if the user inputs a command.
                '''
                # waits for user to input a character to exit help menu
                if self._ser.any():
                    self._state = S0_MM
#============================================================================== 
            elif self._state == S3_KP:
                '''
                State 3: Gain Selection
                The robot is in the gain selection state and waiting for user input.
                The robot will display the gain selection state and wait for user input.
                The robot will then transition to the Ki selection state if the user inputs a number.
                '''
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
                '''
                State 4: Ki Selection
                The robot is in the Ki selection state and waiting for user input.
                The robot will display the Ki selection state and wait for user input.
                The robot will then transition to the main menu if the user inputs a command.
                '''
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
                '''
                State 8: Setpoint Selection
                The robot is in the setpoint selection state and waiting for user input.
                The robot will display the setpoint selection state and wait for user input.
                The robot will then transition to the main menu if the user inputs a command.
                '''
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
                '''
                State 5: Start the Test
                The robot is in the start the test statefor set point control and waiting for user input.
                The robot will display the start the test state and wait for user input.
                The robot will then transition to the wait state until set point control is complete.
                '''
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
                '''
                State 6: Wait
                The robot is in the wait state and waiting for the set point control to be complete. once complete, the robot will transition to the data collection state.
                '''
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
                '''
                State 7: Data Collection
                The robot is in the data collection state and waiting for the data collection to be complete.
                The robot will display the data collection state and wait for the data collection to be complete.
                The robot will then print the data to the serial port and the file.
                The robot will then transition to the main menu once complete.
                '''
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
                '''
                State 9: Line Following Menu
                The robot is in the line following menu and waiting for user input.
                The robot will display the line following menu and then transition to the line follow input state.
                '''
                self._ser.write("+--------------------+\r\n")
                self._ser.write("|Line Following menu\r\n")
                self._ser.write("+---+----------------+\r\n")
                self._ser.write(f"| s | speed selection: current {self._speed}\r\n")
                self._ser.write("| r | return to main menu\r\n")
                self._ser.write("| g | start the test\r\n")
                self._ser.write("+---+----------------+\r\n")
                
                self._ser.write(UI_prompt)
                self._state = S10_LIN
#==============================================================================
            elif self._state == S10_LIN:
                '''
                State 10: Line Follow Input
                The robot is in the line follow input state and waiting for user input.
                The robot will display the line follow input state and wait for user input.
                The robot will then transition to the speed selection state if the user inputs 's'.
                The robot will then transition to the return to main menu state if the user inputs 'r'.
                The robot will then transition to the start the test state if the user inputs 'g'.
                '''
                if self._ser.any():
                    inChar = self._ser.read(1).decode()
                    
                    if inChar in {'s','S'}:
                        self._ser.write("select line follow steady speed\r\n")
                        self._ser.write(UI_prompt)
                        self._state = S11_SETS
                        
                    elif inChar in {'r','R'}:
                        self._line.put(False)
                        self._state = S0_MM
                        
                    elif inChar in {'g','G'}:
                        self._ser.write('Press any to stop\r\n')
                        if self._startTime is not None:
                            self._startTime.put(time.ticks_us())
                        self._leftMotorGo.put(True)
                        self._rightMotorGo.put(True)
                        self._state = S12_HOLD
#==============================================================================
            elif self._state == S11_SETS:
                '''
                State 11: Set Speed
                The robot is in the set speed state and waiting for user input.
                The robot will display the set speed state and then transition back to the line following menu.
                '''
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
                '''
                State 12: Hold
                The robot is in the hold state and waiting for the user to press any button to stop the line following.
                The robot will then transition to the data collection state if the line following is complete and the track flag is false then resturn to the main menu.
                '''
                if self._ser.any():
                    self._ser.read()
                    self._rightMotorGo.put(False)
                    self._leftMotorGo.put(False)
                    if not track_flag:
                        self._collectflag = 1
                        self._ser.write("Starting line follow data collection...\r\n")
                        self._state = S13_DATL
                    else:
                        track_flag = False
                        self._state = S0_MM
#==============================================================================
            elif self._state == S13_DATL:
                '''
                State 13: Data Collection
                The robot is in the data collection state and waiting for the data collection to be complete.
                The robot will then print the data to the serial port and the file.
                The robot will then transition to the main menu once complete.
                '''
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
                '''
                State 14: Track the Line
                The robot is in the track the line state and waiting for the user to press any button to start the line following.
                The robot will then transition to the hold state if the user presses any button.
                '''
                self._ser.write('Press any to stop\r\n')
                if self._startTime is not None:
                    self._startTime.put(time.ticks_us())
                self._leftMotorGo.put(True)
                self._rightMotorGo.put(True)
                track_flag = True
                self._state = S12_HOLD
                
#==============================================================================
            yield self._state