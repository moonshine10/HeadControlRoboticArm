# Head Control Robotic Arm
This code is for MANO robotic arm project. https://www.youtube.com/watch?v=ZL-D6KFdTks

# MANO Robotic ARM
MANO robotic arm is a robotic arm designed for quadriplegic patients for their daily activity. It is controlled by head movement and mouth action include sip, puff and bite. The robotic arm has a claw that can be open and close to assist user with daily activity such as pick up a cup, push a button to open the door for example. The head control set uses various technology such as sensor fusion ( accelerometer, gyroscope , magnetometer) to map users’ head movement and a combination of sensors to map user’s mouse movement.  

# Human Machine Interface

I was the embedded system engineer in Human Machine Interface team for MANO project. Our team is responsible for building a hardware and software interface for user to interact with the system. We designed a **head-controlled Bluetooth mouse** to control the robotic arm. Here is how user's movement was mapped into mouse activity:
+ head’s pitch and roll movement to control mouse’s movement along X and Y access  
+ we designed a mouth controller that user can sip, puff and bite on to map right click, left click and scroll on a BT mouse.
In addition to the mouse functionality, I implemented calibration and sensitivity adjustment. (See PNG file for system design)

# What I did


Personally, I built the all of the embedded software for our team. "headsetControl.c" is the part of the head control software I built. This program demonstrates the following function : 
+ mouse control : a combination of accelerometer, gyroscope , magnetometer will track user's head movement; sip-puff-bite mouth piece will track users' mouth activity. The software will translate the sensor data into Bluetooth mouse command, then package it in UART packets, and sent to the reciever to control robotic arm through Bluetooth as a mouse profile.  

+ calibration mode: user will be able to re-center the device 
+ sensitivity selection 
+ control menu: a control menu is implemented for initialize setting, select different mode ( mouse control mode, sensitivity selection mode, calibration mode). The headset uses LED and buzzer as signifer for menu selection 
