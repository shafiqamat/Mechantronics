from pyb import Pin, Timer
from time import sleep_ms
import motor
from encoder import Encoder
from time import ticks_ms, ticks_diff

#__init__(self, PWM, DIR, nSLP, timernumber, channelnumber):
right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
left_motor = motor.Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)

leftEncoder  = Encoder(3, Pin.cpu.B5, 1, Pin.cpu.B4, 2, direction=-1)
rightEncoder = Encoder(2, Pin.cpu.A0, 1, Pin.cpu.A1, 2, direction=-1)

print('motor on')

def tonb():             #turn on both motors
    right_motor.enable()
    left_motor.enable()

def toffb():            #turn off both motors
    right_motor.disable()
    left_motor.disable()

def sb(effort):         #set both motors to the same effort
    right_motor.set_effort(effort)
    left_motor.set_effort(effort)


def tr(effort):         #turn right motor to the given effort
    right_motor.set_effort(effort)
    left_motor.set_effort(0)

def tl(effort):         #turn left motor to the given effort
    right_motor.set_effort(0)
    left_motor.set_effort(effort)
count = 0
print_timer = ticks_ms()
while True:
    if count == 0:
        leftEncoder.zero()
        rightEncoder.zero()
        left_motor.enable()
        right_motor.enable()
        left_motor.set_effort(100)
        right_motor.set_effort(100)

    leftEncoder.update()
    rightEncoder.update()
    if ticks_diff(ticks_ms(), print_timer) >= 500:
        print_timer = ticks_ms()
        print("left rpm:")
        print(leftEncoder.get_velocity()*1_000_000)
        print("right rpm:")
        print(rightEncoder.get_velocity()*1_000_000)
    count += 1
    sleep_ms(20)