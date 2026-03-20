Design
======

Hardware Design
---------------

The hardware design of the robot is based on the following components:

- 2x DC motors with encoders
- IMU sensor (BNO055)
- IR sensor array for line detection
- STM32 Nucleo board

Software Design
---------------

Each of the above hardware components are controlled using drivers we created

- :ref:'Motor driver <_motor_module>`
- :ref:'Encoder driver <_encoder_module>`
- :ref:'IMU driver <_IMU_driver_module>`
* IR sensor array is a digital sensor array that is implemented directly in the 'controller task <task_controller_module>` rather than a driver.

Implemenation
---------------

The robot integrates these drivers into a Finite State Machine which controls the robot's behavior. These are
the different files needed to implement the robot:

- :ref:'task_user.py <_task_user_module>`: This file creates the UI for the robot and is the main way the
  user interacts with the robot. It is a Finite State Machine that allows the
  user to input commands to the robot.

- :ref:'task_controller.py <_task_controller_module>`: This file is a controller that implements the PI control law and the state estimation law. 

- :ref:'task_observer.py <_task_observer_module>`: This file is the observer that estimates the robot's state usign the IMU and the motor encoders.

- :ref:'task_motor.py <_task_motor_module>`: This file is the motor task that controls the robot's motors using inputs from the controller, the observer and the user input.

How main is set up:

Our main.py file is the entry point for the robot. It initializes all the hardware components and builds the tasks for Romi.
The hardware components are:
- leftMotor
- rightMotor
- leftEncoder
- rightEncoder
- IMU
- (line sensor array is initialized in the controller task)
The Shares and Queues are:

.. list-table::
   :header-rows: 1
   :widths: 18 10 10 38 10

   * - Variable
     - Kind
     - Type
     - Name (constructor argument)
     - Capacity
   * - `setpoint`
     - Share
     - `f`
     - `setpoint`
     - `1`
   * - `gainP`
     - Share
     - `f`
     - `preportional gain`
     - `1`
   * - `gainI`
     - Share
     - `f`
     - `intergral gain`
     - `1`
   * - `R_pos`
     - Share
     - `f`
     - `current right value`
     - `1`
   * - `R_err`
     - Share
     - `f`
     - `current right error`
     - `1`
   * - `L_pos`
     - Share
     - `f`
     - `current left value`
     - `1`
   * - `L_err`
     - Share
     - `f`
     - `current left error`
     - `1`
   * - `speed`
     - Share
     - `f`
     - `line follow steady speed`
     - `1`
   * - `start`
     - Share
     - `B`
     - `start flag`
     - `1`
   * - `line`
     - Share
     - `B`
     - `start line follow`
     - `1`
   * - `current_x`
     - Share
     - `f`
     - `current x position`
     - `1`
   * - `current_y`
     - Share
     - `f`
     - `current y position`
     - `1`
   * - `heading`
     - Share
     - `f`
     - `heading`
     - `1`
   * - `leftMotorGo`
     - Share
     - `B`
     - `Left Mot. Go Flag`
     - `1`
   * - `rightMotorGo`
     - Share
     - `B`
     - `Right Mot. Go Flag`
     - `1`
   * - `startTime`
     - Share
     - `l`
     - `Data collection start time (us)`
     - `1`
   * - `estx`
     - Queue
     - `f`
     - `estimated x position`
     - `400`
   * - `esty`
     - Queue
     - `f`
     - `estimated y position`
     - `400`
   * - `leftDataValues`
     - Queue
     - `f`
     - `Left position data`
     - `40`
   * - `leftTimeValues`
     - Queue
     - `l`
     - `Left time (us)`
     - `40`
   * - `rightDataValues`
     - Queue
     - `f`
     - `Right position data`
     - `40`
   * - `rightTimeValues`
     - Queue
     - `l`
     - `Right time (us)`
     - `40`
   * - `centroidValues`
     - Queue
     - `f`
     - `centroid of line follower`
     - `400`
   * - `centroidTime`
     - Queue
     - `l`
     - `time of line follow data (us)`
     - `400`

We then instantiate the tasks and add them to the task list.
The tasks are:

.. list-table::
   :header-rows: 1
   :widths: 28 10 12

   * - Task name
     - Priority
     - Period (ms)
   * - :ref:`Left Motor Task <_task_motor_module>`
     - `2`
     - `50`
   * - :ref:`Right Motor Task <_task_motor_module>`
     - `2`
     - `50`
   * - :ref:`Controller Task <_task_controller_module>`
     - `2`
     - `50`
   * - :ref:`Observer Task <_task_observer_module>`
     - `1`
     - `20`
   * - :ref:`User Interface Task <_task_user_module>`
     - `0`
     - `0`

You can click on the task name to see what each task does and it's parameters.

Then main.py runs the task list using the cooperative scheduler. which exists when Ctrl-C is pressed.
