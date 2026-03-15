# -*- coding: utf-8 -*-
"""
Created on Wed Mar  4 20:37:25 2026

testing code for the BNO055 IMU

@author: maxwe
"""
from IMU_driver import BNO055
from pyb import I2C
import time 

i2c = I2C(3, I2C.CONTROLLER, baudrate=100000)

print('IMU address: ')
print(hex(i2c.scan()[0]))

IMU = BNO055(i2c,0x28)

time.sleep(1)

IMU.set_mode('NDOF')

time.sleep(1)

print("IMU Mode:")
print(IMU.get_mode())

time.sleep(1)
while True:
    try:
        print(IMU.cal_status())
        IMU.read_cal_coeffs()
        time.sleep_ms(800)
    except KeyboardInterrupt:
        break
  
    
cal_coeffs = IMU.read_cal_coeffs()
print(cal_coeffs)
print("Saving data to 'calibration.txt'")
try:
    with open('calibration.txt','w') as file:
        file.write(str(cal_coeffs))
        print("Data saved sucessfully")
        print("End of line")
except OSError:
    print("If you wish to continue delete existing 'calibration.txt'")
    print("End of line")
    


    
    
    
# while True:
#     try:
#         heading = IMU.heading()
#         yaw     = IMU.yaw_rate()
#         print(f"Heading: {heading} | Yaw Rate: {yaw}")
#         time.sleep_ms(100)
#     except KeyboardInterrupt:
#         break

