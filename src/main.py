from motor        import Motor
from encoder      import Encoder
from IMU_driver   import BNO055
from task_motor   import task_motor
from task_user    import task_user
from task_controller import task_controller
from task_observer import task_observer
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
import gc
from pyb          import Pin, Timer
from pyb import I2C



def print_ram_usage(label="RAM"):
    """Print current heap usage (for debugging MemoryError)."""
    gc.collect()
    alloc = gc.mem_alloc()
    free = gc.mem_free()
    total = alloc + free
    pct = (100 * alloc // total) if total else 0
    print(f"{label}: alloc={alloc} free={free} total={total} ({pct}% used)")

# Build all driver objects first
leftMotor    = Motor(Pin.cpu.B8, Pin.cpu.B15, Pin.cpu.B14, 4, 3)
rightMotor   = Motor(Pin.cpu.C8, Pin.cpu.C6, Pin.cpu.C5, 8, 3)
# direction=-1 so positive motion gives positive position (both encoders were inverted)
leftEncoder  = Encoder(3, Pin.cpu.B5, 1, Pin.cpu.B4, 2, direction=-1)
rightEncoder = Encoder(2, Pin.cpu.A0, 1, Pin.cpu.A1, 2, direction=-1)

i2c = I2C(3, I2C.CONTROLLER, baudrate=100000)
IMU = BNO055(i2c, 0x28)

# Build shares and queues
setpoint =  Share("f", name="setpoint")
gainP    =  Share("f", name="preportional gain")
gainI    =  Share("f", name="intergral gain")
R_pos    =  Share("f", name="current right value")
R_err    =  Share("f", name="current right error")
L_pos    =  Share("f", name="current left value")
L_err    =  Share("f", name="current left error")

speed    =  Share("f", name="line follow steady speed")

start    =  Share("B", name='start flag')
line   =  Share("B", name="start line follow")

estx   = Queue("f",400,name='estimated x position')
esty   = Queue("f",400,name='estimated y position')
current_x = Share("f", name='current x position')
current_y = Share("f", name='current y position')
heading = Share("f", name='heading')

leftMotorGo   = Share("B",     name="Left Mot. Go Flag")
rightMotorGo  = Share("B",     name="Right Mot. Go Flag")
# One motor is tested at a time (never both). Separate queues per motor for position + time.
startTime        = Share("l", name="Data collection start time (us)")
leftDataValues   = Queue("f", 40, name="Left position data")
leftTimeValues   = Queue("l", 40, name="Left time (us)")
rightDataValues  = Queue("f", 40, name="Right position data")
rightTimeValues  = Queue("l", 40, name="Right time (us)")

centroidValues   = Queue("f", 400, name='centroid of line follower')
centroidTime     = Queue("l", 400, name='time of line follow data (us)')

# Build task class objects
leftMotorTask  = task_motor(leftMotor,  leftEncoder, start, startTime, line,
                            L_pos, L_err, leftMotorGo, leftDataValues, leftTimeValues)
rightMotorTask = task_motor(rightMotor, rightEncoder, start, startTime, line,
                            R_pos, R_err, rightMotorGo, rightDataValues, rightTimeValues)

controllerTask = task_controller(gainP, gainI, setpoint, speed,start,line,
                                 R_pos, R_err, L_pos, L_err,centroidValues, centroidTime, estx, esty, current_x, current_y, heading)

observerTask  = task_observer(IMU, R_pos, R_err, L_pos, L_err, estx,esty, start, current_x, current_y, heading)

userTask = task_user(setpoint, gainP, gainI, speed, line, leftMotorGo, rightMotorGo,
                     leftDataValues, leftTimeValues, rightDataValues, rightTimeValues, centroidValues, centroidTime,
                     estx, esty,startTime)
                     
# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 2, period = 50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 2, period = 50, profile=True))
task_list.append(Task(controllerTask.run, name="Cont. Task",
                      priority = 2, period = 50, profile=True))
task_list.append(Task(observerTask.run, name="Obsv. Task",
                      priority = 1, period = 20, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))

# Run the garbage collector preemptively
gc.collect()
print_ram_usage("RAM at startup")

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
    except MemoryError:
        print("MemoryError - RAM usage at failure:")
        print_ram_usage("  heap")
        raise
    except KeyboardInterrupt:
        print("Program Terminating")
        print("MemoryError - RAM usage at failure:")
        print_ram_usage("  heap")
        leftMotor.disable()
        rightMotor.disable()
        break

print("\n")
print(task_list)
print(show_all())