import pyb, utime, struct
#from bno055_driver import BNO055

# i2c = pyb.I2C(3, pyb.I2C.CONTROLLER, baudrate=400000)

# # bno055_driver.py
# address = hex(i2c.scan())

#IMU =BNO055(i2c, address=address)

class BNO055:


    def __init__(self, i2c, address):
        """
        Initializes an IMU object on the given I2C object and address.
        :param: i2c (The I2C object to use for the IMU)
        :param type: I2C object
        :param: address (The address of the IMU)
        :param type: int
        :notes: create a bytearray of the registers and the operation modes used in all the functions.
            i2c (I2C): The I2C object to use for the IMU.
            address (int): The address of the IMU.
        Notes: create a bytearray of the registers and the operation modes used in all the functions.
        Registers:
        - REG_CHIP_ID: 0x00 (chip ID register)
        - REG_OPR_MODE: 0x3D (operation mode register)
        - REG_PWR_MODE: 0x3E (power mode register)
        - REG_SYS_TRIGGER: 0x3F (system trigger register)
        - REG_PAGE_ID: 0x07 (page ID register)
        - REG_CALIB_STAT: 0x35 (calibration status register)
        - EUL_Heading_LSB: 0x1A (heading register)
        - EUL_Roll_LSB: 0x1C (roll register)
        - EUL_Pitch_LSB: 0x1E (pitch register)
        - GYR_DATA_X_LSB: 0x14 (gyroscope data X register)
        - GYR_DATA_Z_LSB: 0x18 (gyroscope data Z register)
        - ACC_DATA_X_LSB: 0x08 (accelerometer data X register)
        - LIA_DATA_X_LSB: 0x28 (linear accelerometer data X register)
        - ACC_LSB_PER_MPS2: 100.0 (scale factor for BNO055 in m/s^2 mode)
        - REG_CALIB_START: 0x55 (calibration start register)
        - CALIB_LEN: 22 (length of the calibration coefficients block)
        Operation modes:
        - OPR_MODE: 0x3D (register for setting operation mode)
        - MODE_CONFIG: 0x00 (config mode)
        - MODE_IMU: 0b00001000 (IMU mode)
        - MODE_COMP: 0b00001001 (compass mode)
        - MODE_M4G: 0b00001010 (magnet for gyroscope mode)
        - MODE_NDOFoff: 0b00001011 (NDOF off mode)
        - MODE_NDOF: 0b00001100 (NDOF mode)
        Power modes:
        - PWR_NORMAL: 0x00 (normal power mode)
        """
        # -------- Registers (subset you need) --------
        self.REG_CHIP_ID       = 0x00
        self.REG_OPR_MODE      = 0x3D
        self.REG_PWR_MODE      = 0x3E
        self.REG_SYS_TRIGGER   = 0x3F
        self.REG_PAGE_ID       = 0x07
    
        self.REG_CALIB_STAT    = 0x35
    
        self.EUL_Heading_LSB   = 0x1A  # heading/ yaw
        self.EUL_Roll_LSB      = 0x1C
        self.EUL_Pitch_LSB     = 0x1E
        self.GYR_DATA_X_LSB    = 0x14
        self.GYR_DATA_Z_LSB    = 0x18
        self.ACC_DATA_X_LSB    = 0x08  # raw accelerometer vector
        self.LIA_DATA_X_LSB    = 0x28  # linear accel (gravity removed)
        self.ACC_LSB_PER_MPS2  = 100.0 # template scale for BNO055 in m/s^2 mode
    
        # Calibration coefficients block (22 bytes)
        self.REG_CALIB_START   = 0x55
        self.CALIB_LEN         = 22
        
    
        # -------- Operation modes --------
        self.OPR_MODE    = 0x3D   #register for setting operation mode
        self.MODE_CONFIG = 0x00
        self.MODE_IMU    = 0b00001000   # Accel+Gyro fusion (no mag)
        self.MODE_COMP   = 0b00001001   # Compass mode value
        self.MODE_M4G    = 0b00001010   # Magnet for gyroscope mode       
        self.MODE_NDOFoff= 0b00001011   # Accel+Gyro
        self.MODE_NDOF   = 0b00001100   # Same as above with fast magnetometer
    
        # -------- Power modes --------
        self.PWR_NORMAL  = 0x00
        
        
        self.i2c = i2c
        self.address = address

#==============================================================================
    def set_mode(self, mode):
        '''
        Takes in a string and sets the mode of the IMU to one of the 5 fusion modes

        :param: mode (The mode to set the IMU to)
        :param type: str
        :raises: ValueError: If mode is not a valid fusion mode on IMU (CONFIG, IMU, COMPASS, M4G, NDOF, NDOFoff).
        :returns: None
        '''
        
        if mode == 'CONFIG':
            self.i2c.mem_write(self.MODE_CONFIG,self.address,self.OPR_MODE)
        elif mode == 'IMU':
            self.i2c.mem_write(self.MODE_IMU,self.address,self.OPR_MODE)
        elif mode == 'COMPASS':
            self.i2c.mem_write(self.MODE_COMP,self.address,self.OPR_MODE)
        elif mode == 'M4G':
            self.i2c.mem_write(self.MODE_M4G,self.address,self.OPR_MODE)
        elif mode == 'NDOF':
            self.i2c.mem_write(self.MODE_NDOF,self.address,self.OPR_MODE)
        elif mode == 'NDOFoff':
            self.i2c.mem_write(self.MODE_NDOFoff,self.address,self.OPR_MODE)
        else:
            raise ValueError
        return
#==============================================================================
    def get_mode(self):
        '''
        Returns the current mode of the IMU.
        :param: None     
        :returns: mode (The current mode of the IMU)
        :return type: str
        '''
        buff = bytearray(1)
        self.i2c.mem_read(buff,self.address,self.OPR_MODE)
        hexmode = buff[0]
        if hexmode == self.MODE_CONFIG:
            mode = 'CONFIG'
        elif hexmode == self.MODE_IMU:
            mode = 'IMU'
        elif hexmode == self.MODE_IMU:
            mode = 'IMU'
        elif hexmode == self.MODE_COMP:
            mode = 'COMP'
        elif hexmode == self.MODE_M4G:
            mode = 'M4G'
        elif hexmode == self.MODE_NDOF:
            mode = 'NDOF'
        elif hexmode == self.MODE_NDOFoff:
            mode = 'NDOFoff'
        return mode
#==============================================================================
    def cal_status(self):
        '''
        returns a string that classifies the current calibrations status of the 
        IMU. Call print on this function to print to terminal, or check to see
        if status has certian strings inside of it to calibrate without printing

        :param: None
        :returns: status (The current calibration status of the IMU)
        :return type: str
        :notes:
        - SYSTEM CALIBRAITION COMPLETE: All systems are calibrated.
        - SYSTEM PARTLY CALIBRATED: Some systems are calibrated.
        - SYSTEM NOT CALIBRAITED: No systems are calibrated.
        '''
        status = ''
        buff = bytearray(1)
        self.i2c.mem_read(buff,self.address,self.REG_CALIB_STAT)
        cal = buff[0]>>6 & 0b11
        gyro= buff[0]>>4 & 0b11
        acc = buff[0]>>2 & 0b11
        mag = buff[0]    & 0b11
        #calibration status
        if cal == 3:
            status += 'SYSTEM CALIBRAITION COMPLETE\r\n'
        elif cal in {1,2}:
            status += 'SYSTEM PARTLY CALIBRATED\r\n'
        else:
            status += 'SYSTEM NOT CALIBRAITED\r\n'
        #GYRO Status
        if gyro == 3:
            status += 'GYRO CALIBRAITION COMPLETE\r\n'
        elif gyro in {1,2}:
            status += 'GYRO PARTLY CALIBRATED\r\n'
        else:
            status += 'GYRO NOT CALIBRAITED\r\n'
        #ACC Status
        if acc == 3:
            status += 'ACC CALIBRAITION COMPLETE\r\n'
        elif acc in {1,2}:
            status += 'ACC PARTLY CALIBRATED\r\n'
        else:
            status += 'ACC NOT CALIBRAITED\r\n'
        #MAG Status
        if mag == 3:
            status += 'MAGNETOMETER CALIBRAITION COMPLETE\r\n'
        elif mag in {1,2}:
            status += 'MAGNETOMETER PARTLY CALIBRATED\r\n'
        else:
            status += 'MAGNETOMETER NOT CALIBRAITED\r\n'
        status += '----------------------\r\n'
        return status
#==============================================================================
    def read_cal_coeffs(self):
        '''
        reads the calibration coefficents of the IMU and puts them into a tuple
        that has order accX,accY,accZ,magX,magY,magZ,gyoX,gyoY,gyoZ,accR,magR

        :param: None
            None
        :returns: values (The calibration coefficients of the IMU)
        :return type: tuple
        :notes: tuple returned - (accX,accY,accZ,magX,magY,magZ,gyoX,gyoY,gyoZ,accR,magR)
        '''
        
        buff = bytearray(22)
        self.i2c.mem_read(buff,self.address,self.REG_CALIB_START)
        
        accX,accY,accZ,magX,magY,magZ,gyoX,gyoY,gyoZ,accR,magR = struct.unpack('<hhhhhhhhhhh',buff)
        
        values = accX,accY,accZ,magX,magY,magZ,gyoX,gyoY,gyoZ,accR,magR
        return values
#==============================================================================
    def write_cal_coeffs(self, blob):
        '''
        writes calibration coefficents into the imu
        :param: blob (The calibration coefficients of the IMU)
        :param type: tuple
        :returns: None
        :notes: tuple returned - (accX,accY,accZ,magX,magY,magZ,gyoX,gyoY,gyoZ,accR,magR)
        '''
        
        values = bytearray(22)
        count = 0
        # takes each element in blob and reverses the bit order and adds it to
        #the values bytearray so that LSB is first so the registers line up
        while count < len(blob):
            temp_bytes = blob[count]
            temp_bytes.reverse()
            
            values += temp_bytes
            
            count += 1
    
        self.i2c.mem_write(values,self.address,self.REG_CALIB_START)
        return
#==============================================================================
    def euler(self):
        '''
        Returns the Euler angles of the IMU.

        :Param: None
        :Returns: angles (The Euler angles of the IMU)
        :Return type: tuple
        :Notes: tuple returned - (heading,roll,pitch)
        '''
        buff = bytearray(6)
        self.i2c.mem_read(buff,self.address,self.EUL_Heading_LSB)
        
        heading,roll,pitch = struct.unpack('<hhh',buff)        
        
        angles = heading/16,roll/16,pitch/16
        return angles
#==============================================================================
    def get_ang_velocity(self):
        '''
        Returns the angular velocity of the IMU.
        :param: None
        :returns: omegas (The angular velocity of the IMU)
        :return type: tuple
        :notes: tuple returned - (gyrX,gyrY,gyrZ)
        '''
        buff = bytearray(6)
        self.i2c.mem_read(buff,self.address,self.GYR_DATA_X_LSB)
        
        gyrX,gyrY,gyrZ = struct.unpack('<hhh',buff)
        
        omegas = gyrX/16,gyrY/16,gyrZ/16
        return omegas
#==============================================================================
    def heading(self):
        '''
        Returns the heading of the IMU.
        :param: None
        :returns: heading (The heading of the IMU)
        :return type: int
        :notes: heading is in degrees.
        '''
        buff = bytearray(2)
        self.i2c.mem_read(buff,self.address,self.EUL_Heading_LSB)
        
        return struct.unpack('<h',buff)[0]/16
#==============================================================================
    def yaw_rate(self):
        '''
        Returns the yaw rate of the IMU.
        :param: None
        :returns: yaw (The yaw rate of the IMU)
        :return type: int
        :notes: yaw is in degrees/second.
        '''
        buff = bytearray(2)
        self.i2c.mem_read(buff,self.address,self.GYR_DATA_Z_LSB)
        yaw = struct.unpack('<h',buff)[0]/16
        return yaw
#==============================================================================
    def get_accel_raw(self):
        '''
        Returns the raw acceleration of the IMU.
        :param: None
        :returns: accels (The raw acceleration of the IMU)
        :return type: tuple
        :notes: tuple returned - (accX,accY,accZ) each 16bit signed int raw register value.
        '''
        buff = bytearray(6)
        self.i2c.mem_read(buff,self.address,self.ACC_DATA_X_LSB)

        accX,accY,accZ = struct.unpack('<hhh',buff)
        return accX,accY,accZ
#==============================================================================
    def get_linear_accel_raw(self):
        '''
        Returns the linear acceleration of the IMU (gravity compensated by fusion).

        :param: None
        :returns: lin_accels (The linear acceleration of the IMU)
        :return type: tuple
        :notes: tuple returned - (linX,linY,linZ) each 16bit signed int raw value.
        '''
        buff = bytearray(6)
        self.i2c.mem_read(buff,self.address,self.LIA_DATA_X_LSB)

        linX,linY,linZ = struct.unpack('<hhh',buff)
        return linX,linY,linZ
#==============================================================================
    def get_accel_mps2(self):
        '''
        Returns the unit-converted acceleration of the IMU.

        :param: None
        :returns: lin_accels (The linear acceleration of the IMU)
        :return type: tuple
        :notes: tuple returned - (accX,accY,accZ) each an integer in m/s^2. Due to orientation of the IMU on the robot X and Y are switched.
        '''
        accX,accY,accZ = self.get_accel_raw()
        return (accY/self.ACC_LSB_PER_MPS2, #switch x and y to account for the orientation of the IMU
                accX/self.ACC_LSB_PER_MPS2,
                accZ/self.ACC_LSB_PER_MPS2)
#==============================================================================
    def get_linear_accel_mps2(self):
        '''
        Returns the unit-converted linear acceleration of the IMU.
        :param: None
        :returns: lin_accels (The linear acceleration of the IMU)
        :return type: tuple
        :notes: tuple returned - (linx,liny,linz) each an integer in m/s^2. Due to orientation of the IMU on the robot X and Y are switched.
        '''
        linX,linY,linZ = self.get_linear_accel_raw()
        return (linY/self.ACC_LSB_PER_MPS2, #switch x and y to account for the orientation of the IMU
                linX/self.ACC_LSB_PER_MPS2,
                linZ/self.ACC_LSB_PER_MPS2)
#==============================================================================

#IMU.set_mode('IMU')     # IMU fusion (no mag) — usually best near motors
# imu.set_mode(BNO055.MODE_NDOF)  # if you want magnetometer

# while True:
#     print("heading:", IMU.heading(), "yaw_rate:", IMU.yaw_rate(), "cal:", IMU.cal_status())
#     utime.sleep_ms(100)
