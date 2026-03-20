Design
======

Hardware Design
---------------

The hardware design of the robot is based on the following components:

- 2x DC motors with encoders
- IMU sensor(BNO055)
- IR sensor array for line detection
- STM32 Nucleo board

Software Design
---------------

Each of the above hardware components are controlled using drivers we created

- Motor driver
- Encoder driver
- IMU driver
- IR sensor array driver

Implemenation
---------------

The robot integrates these drivers into a Finite State Machine which controls the robot's behavior. These are
the different files needed to implement the robot:

- task_user.py: This file is creates the UI for the robot and is the main way the user interacts with the robot. 
It is a Finite State Machine that allows the user to input commands to the robot.

- task_controller.py: This file is a controller that implements the PI control law and the state estimation law. 

- task_observer.py: This file is the observer that estimates the robot's state usign the IMU and the motor encoders.

- task_motor.py: This file is the motor task that controls the robot's motors using inputs from the controller, the observer and the user input.




