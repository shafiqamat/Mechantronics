Results
=======

Goals
-----
- To follow a line using:
    - PI control
    - IR sensor array
- To avoid obstacles and navigate a maze without using lines using state estimation and the following hardware:
    - IMU sensor(BNO055)
    - Motor encoders

Initial Line Following Testing:
-------------------------------

We initially tested the line following capabilities of the robot by following a circular track. The user
would input a speed and a gain for the PI controller for line following. The robot would then follow a line at the given 
speed and gain.

.. image:: _static/circulartrack.jpg
   :alt: Circular Track Photo
   :width: 500px

.. video:: _static/Romi_circle_follow.mp4

   :width: 500px
   :autoplay:
   :nocontrols:

Final Track to Navigate:

We then tested the line following capabilities of the robot by following the given track. 
We gave the robot a gain of 5 for Kp and 0.3 for Ki for the PI controller for line following. 
For state estimation we utilized a PID controller, with the following gains: 0.5 
for Kp, 0.05 for Ki and .2 for Kd for the heading state estimation. and Kd = 1, Ki = 0.05 and Kd = .1 for the forward state estimation.


.. image:: _static/Game_Track.png 
   :alt: Track to navigate Photo
   :width: 500px

Final Video
-----------
.. video:: _static/finalvideo.mp4

   :width: 500px
   :autoplay:
   :nocontrols:
