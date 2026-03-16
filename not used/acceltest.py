from IMU_driver import BNO055
import time
from pyb import I2C

i2c = I2C(3, I2C.CONTROLLER, baudrate=100000)
IMU = BNO055(i2c,0x28)
IMU.set_mode('NDOF')
print("start")
while True:
    try: 
        if IMU.get_linear_accel_mps2()[0] > 3 or IMU.get_linear_accel_mps2()[0] < -3:
            print("x: ", IMU.get_linear_accel_mps2()[0])
            print("line_accel:", IMU.get_linear_accel_mps2(), "accel:", IMU.get_accel_mps2())
        if IMU.get_linear_accel_mps2()[1] > 3 or IMU.get_linear_accel_mps2()[1] < -3:
            print("y: ", IMU.get_linear_accel_mps2()[1])
            print("line_accel:", IMU.get_linear_accel_mps2(), "accel:", IMU.get_accel_mps2())
        if IMU.get_linear_accel_mps2()[2] > 3 or IMU.get_linear_accel_mps2()[2] < -3:
            print("z: ", IMU.get_linear_accel_mps2()[2])
            print("line_accel:", IMU.get_linear_accel_mps2(), "accel:", IMU.get_accel_mps2())
    except KeyboardInterrupt:
        break
    time.sleep_ms(50)
