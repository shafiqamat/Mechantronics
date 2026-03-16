# -*- coding: utf-8 -*-
"""
Smooth line following with 5-sensor array.
- PID control from weighted line position (P + I + D).
- Optional low-pass smoothing on error to reduce jerk.
- Tunable base speed, gains (Kp, Ki, Kd), and smoothing at top of file.
"""
import time
from pyb import Pin
import motor

right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
left_motor = motor.Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)

right_motor.enable()
left_motor.enable()

# Line sensor pins (sensor sees line when value is 0 / low)
center = Pin(Pin.cpu.B6, mode=Pin.IN)
left = Pin(Pin.cpu.A7, mode=Pin.IN)
right = Pin(Pin.cpu.C7, mode=Pin.IN)
fleft = Pin(Pin.cpu.A6, mode=Pin.IN)
fright = Pin(Pin.cpu.A9, mode=Pin.IN)

# Tuning: base speed and PID gains
BASE_EFFORT = 50
Kp = 20
KP = Kp * (BASE_EFFORT / 20)
KI = 0.5
Kd = 2.0
KD = Kd * (5*BASE_EFFORT)
# Loop period (s) for integral and derivative
DT = 0.025
# Optional: smooth error (0 = no smoothing, 0.9 = very smooth)
SMOOTH = 0.9


def read_line_position():
    """Return weighted line position in [-2, 2]. 0 = centered, negative = line left."""
    # Weights: far left -2, left -1, center 0, right +1, far right +2
    pos = 0.0
    n = 0
    if not fleft.value():
        pos -= 2
        n += 1
    if not left.value():
        pos -= 1
        n += 1
    if not center.value():
        n += 1
    if not right.value():
        pos += 1
        n += 1
    if not fright.value():
        pos += 2
        n += 1
    if n == 0:
        return None
    return pos / max(n, 1)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# Driving loop
while True:
    right_motor.set_effort(0)
    left_motor.set_effort(0)
    input('Enter anything to start\r\n')

    err_smooth = 0.0
    err_prev = 0.0
    integral = 0.0
    try:
        while True:
            pos = read_line_position()
            if pos is None:
                err = 0.0
            else:
                err = pos

            err_smooth = SMOOTH * err_smooth + (1 - SMOOTH) * err

            integral += err_smooth * DT
            derivative = (err_smooth - err_prev) / DT
            err_prev = err_smooth

            steer = KP * err_smooth + KI * integral + KD * derivative
            left_effort = clamp(BASE_EFFORT + steer, -100, 100)
            right_effort = clamp(BASE_EFFORT - steer, -100, 100)

            if (BASE_EFFORT + steer) != left_effort or (BASE_EFFORT - steer) != right_effort:
                integral -= err_smooth * DT

            left_motor.set_effort(int(left_effort))
            right_motor.set_effort(int(right_effort))
            time.sleep_ms(int(DT * 1000))
    except KeyboardInterrupt:
        continue
