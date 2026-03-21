Project Overview
================

This project implements a line-following robot using a MicroPython-based 
control system on an STM32 microcontroller.

Authors
-------
- Max Soury
- Shafiq Amat

Project Date
------------
January - March 2026

.. image:: _static/romi.jpg
   :alt: Robot Photo
   :width: 500px

How our robot works:
--------------------
Romi utilizes a PID controller with a digital IR sensor array to follow a line and then uses state 
estimation to avoid obstacles and navigate a maze without using lines. 

Hardware
--------

The robot consists of the following components:

- 2x DC motors with encoders
- IMU sensor (BNO055) 
- IR sensor array for line detection
- STM32 Nucleo board

Final Video
----------

.. video:: _static/finalvideo.mp4

   :width: 500px
   :autoplay:
   :nocontrols: