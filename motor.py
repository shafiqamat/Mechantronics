from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self, PWM, DIR, nSLP, timernumber, channelnumber):
        '''Initializes a Motor object'''
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP)
        self.PWM_pin = PWM
        self.DIR_pin = Pin(DIR, mode=Pin.OUT)
        self.channelnumber = channelnumber
        self.timernumber = timernumber        
        self.tim = Timer(self.timernumber, freq=20000)
        self.nSLP_pin.low()
        self.channel = self.tim.channel(self.channelnumber, pin=self.PWM_pin, mode=Timer.PWM, pulse_width_percent=0)


    
    def set_effort(self, effort):
        '''Sets the present effort requested from the motor based on an input value
           between -100 and 100'''
        if effort > 100:
            effort = 100 
        elif effort < -100:
            effort = -100
        elif effort == 0:
            self.channel.pulse_width_percent(0)
            return
        elif effort > 0:
            self.DIR_pin.low()
        else:
            effort = -effort # set effort back to positive and switch the direction pin
            self.DIR_pin.high()
        self.channel.pulse_width_percent(effort)   
        # print(f"effort set to {effort}")          
        pass
            
    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.nSLP_pin.high()
        self.channel.pulse_width_percent(0)
        # print("Motor enabled")
        pass
            
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()
        # print("Motor disabled")