# speed_tune.py
# Standalone MicroPython speed PI tuning script for Romi motors + encoders
#
# Workflow:
# 1) Boot board, Ctrl-C out of main if needed
# 2) Run: import speed_tune
# 3) When prompted, enter target RPM, KP, KI
# 4) Watch prints: ref RPM, measured RPM, efforts
# 5) Tune:
#    - Set KI = 0 first, raise KP until response is quick but not oscillatory
#    - Then raise KI until steady-state error goes near zero (avoid slow hunting)

from pyb import Pin
from time import sleep_ms, ticks_us, ticks_diff, ticks_ms
import motor
from encoder import Encoder

# ---------------- Hardware mapping (matches your main) ----------------
right_motor = motor.Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
left_motor  = motor.Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)

left_enc  = Encoder(3, Pin.cpu.B5, 1, Pin.cpu.B4, 2, direction=-1)
right_enc = Encoder(2, Pin.cpu.A0, 1, Pin.cpu.A1, 2, direction=-1)

# ---------------- Encoder calibration ----------------
CPR_WHEEL = 1444.0  # counts per WHEEL revolution (your measured value)

def rpm_to_cps(rpm):
    return (rpm / 60.0) * CPR_WHEEL

def cps_to_rpm(cps):
    return (cps / CPR_WHEEL) * 60.0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def enc_cps(enc):
    """
    Returns wheel speed in counts/sec using encoder's latest update().
    Uses enc.direction so "forward" can be made positive consistently.
    """
    # enc.delta: counts since last update
    # enc.dt: microseconds since last update
    if enc.dt <= 0:
        return 0.0
    return (enc.direction * enc.delta) * 1_000_000.0 / enc.dt

# ---------------- Test settings ----------------
EFF_LIM = 100

# Step profile times (seconds)
T_REST1 = 1.0   # 0 rpm
T_STEP  = 3.0   # target rpm
T_REST2 = 2.0   # 0 rpm
# Total period = 6 seconds

PRINT_PERIOD_MS = 250  # print rate (ms)
UPDATE_MS = 20         # encoder/controller update rate (ms)

# ---------------- Main tuning routine ----------------
def run():
    print("\n--- Speed PI Tuning ---")
    print("CPR_WHEEL =", CPR_WHEEL)
    print("This test repeats: 0 RPM for {:.1f}s -> target for {:.1f}s -> 0 RPM for {:.1f}s\n"
          .format(T_REST1, T_STEP, T_REST2))

    # Get user inputs (simple + works in MicroPython REPL)
    try:
        target_rpm = float(input("Target RPM (e.g. 40): "))
    except:
        target_rpm = 40.0
        print("Using default Target RPM =", target_rpm)

    try:
        KP_V = float(input("KP_V (start ~0.03 to 0.10): "))
    except:
        KP_V = 0.06
        print("Using default KP_V =", KP_V)

    try:
        KI_V = float(input("KI_V (start 0, then ~0.003 to 0.02): "))
    except:
        KI_V = 0.0
        print("Using default KI_V =", KI_V)

    V_MAX_RPM = 9999.0
    try:
        V_MAX_RPM = float(input("Optional max RPM clamp (enter for none, e.g. 80): ") or "9999")
    except:
        V_MAX_RPM = 9999.0

    v_ref_step = rpm_to_cps(target_rpm)
    v_ref_max = rpm_to_cps(V_MAX_RPM) if V_MAX_RPM < 9000 else 1e9

    print("\nRunning test... Ctrl-C to stop.\n")

    # Enable motors and zero encoders
    right_motor.enable()
    left_motor.enable()
    left_enc.zero()
    right_enc.zero()

    # Integrators (effort units)
    iL = 0.0
    iR = 0.0

    t_start = ticks_us()
    last_print = ticks_ms()

    # For safety, start stopped
    left_motor.set_effort(0)
    right_motor.set_effort(0)
    sleep_ms(200)

    V_FILT = 0.7
    vL_f = 0.0
    vR_f = 0.0

    try:
        while True:
            # Update encoders
            left_enc.update()
            right_enc.update()

            # Measured speeds
            vL = enc_cps(left_enc)
            vR = enc_cps(right_enc)
            vL_f = V_FILT*vL_f + (1 - V_FILT)*vL
            vR_f = V_FILT*vR_f + (1 - V_FILT)*vR


            # Time in current cycle
            t = ticks_diff(ticks_us(), t_start) * 1e-6  # seconds

            # Determine reference speed for this phase
            if t < T_REST1:
                v_ref = 0.0
            elif t < (T_REST1 + T_STEP):
                v_ref = v_ref_step
            elif t < (T_REST1 + T_STEP + T_REST2):
                v_ref = 0.0
            else:
                # restart cycle
                t_start = ticks_us()
                # Optional: clear integrators at cycle restart to compare clean steps
                iL = 0.0
                iR = 0.0
                v_ref = 0.0

            # Clamp reference if desired
            v_ref = clamp(v_ref, -v_ref_max, v_ref_max)

            # PI control (per wheel)
            dt = left_enc.dt * 1e-6
            if dt <= 0:
                dt = UPDATE_MS * 1e-3

            eL = v_ref - vL
            eR = v_ref - vR

            iL += eL * dt
            iR += eR * dt

            FF_PER_RPM = 0.42  # %effort per RPM (start here, tweak)
            eff_ff = FF_PER_RPM * cps_to_rpm(v_ref)   # since v_ref is cps

            effL = eff_ff + KP_V * eL + KI_V * iL
            effR = eff_ff + KP_V * eR + KI_V * iR

            effL_c = clamp(effL, -EFF_LIM, EFF_LIM)
            effR_c = clamp(effR, -EFF_LIM, EFF_LIM)

            # Anti-windup: if clamped, undo the integrator step
            if effL != effL_c:
                iL -= eL * dt
            if effR != effR_c:
                iR -= eR * dt

            left_motor.set_effort(int(effL_c))
            right_motor.set_effort(int(effR_c))

            # Print at a slower rate
            now_ms = ticks_ms()
            if ticks_diff(now_ms, last_print) >= PRINT_PERIOD_MS:
                last_print = now_ms
                print("ref:{:6.1f} rpm | L:{:6.1f} rpm R:{:6.1f} rpm | effL:{:4d} effR:{:4d}"
                      .format(cps_to_rpm(v_ref), cps_to_rpm(vL), cps_to_rpm(vR),
                              int(effL_c), int(effR_c)))

            sleep_ms(UPDATE_MS)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        left_motor.set_effort(0)
        right_motor.set_effort(0)
        left_motor.disable()
        right_motor.disable()

# Auto-run if you execute the file directly with execfile, otherwise call speed_tune.run()
if __name__ == "__main__":
    run()
