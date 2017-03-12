# Head Control Robotic Arm
This code is for MANO robotic arm project. https://www.youtube.com/watch?v=ZL-D6KFdTks

# MANO Robotic ARM
MANO robotic arm is a robotic arm designed for quadriplegic patients for their daily activity. It is controlled by head movement and mouth action include sip, puff and bite. The robotic arm has a claw that can be open and close to assist user with daily activity such as pick up a cup, push a button to open the door for example. The head control set uses various technology such as sensor fusion ( accelerometer, gyroscope , magnetometer) to map users’ head movement and a combination of sensors to map user’s mouse movement.  

# What I did:

I was the embedded system engineer in Human Machine Interface team for MANO project. Our team is responsible for building interface for user to interact with the machine. We come up with the idea of using a head-controlled Bluetooth mouse to control the robotic arm. The head controller will map user head’s pitch and roll movement to control mouse’s movement along X and Y access, and we designed a mouth controller that user can sip, puff and bite on to map right click, left click and scroll on a BT mouse. In addition, calibration and sensitivity adjustment was also implemented on the headset. (See PNG file for system design)

Personally, I built the embedded software for our team. "headsetControl.c" is the part of the head control software I built. This program will take the head movement data tracked by accelerometer, gyroscope , magnetometer; as well as the mouth movement data tracked by pressure sensor, bite sensor, then process the data. The data then got parsed to UART packets, and sent to the reciever to control robotic arm through Bluetooth as a mouse profile.  

In addition, different control modes and mode selection menu were implemented in the code: including calibration mode (re-center), sensitivity selection mode,reset mode, and mouse control mode. User will be able to change mode through a combination of mouth movement. LED and Buzzer are implemented as signifier for mode switching and error warning. 
