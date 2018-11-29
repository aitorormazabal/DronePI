#DronePI

This is a simple quadcopter flight controller for the Raspberry Pi. Here is an example of a quadcopter running this software: https://www.youtube.com/watch?v=fkcsy-yT_t8&feature=youtu.be

The program needs a running ServoBlaster daemon to communicate with the ESCs, and it writes PWM outputs to /dev/servoblaster. You can edit Servo::writeMicroseconds to write the outputs in some other way. The pins corresponding to the front right, front left, back right and back left motors are assumed to be 0,2,1 and 3 respectively by default, and they are set when calling Output::Init in FC::Init.

The control algorithm uses independent cascaded PIDs for each axis. The PID values are read from the Settings.txt file, which has the following format: PWM resolution (same used in ServoBlaster), min and max PWM values in the first three lines, and the rate and angle P I D values for each axis in the following three lines. The order of the axes is roll/pitch/yaw.

Input is received through a UNIX socket created by the program at startup, in the form of 4 byte udp packets. I use a separate python program to receive commands from an Android app and send them to the flight controller through the socket. The first byte of each packet indicates the command type, and the next three the data. These commands can be used for navigating the drone or to change it's parameters on the fly. You can change Input::Init and Input::Update to modify how input is recieved.

I used MPU6050 and MPU9150 IMUs with this software, the code used to interface with the IMU is in IMU.cpp.


Credit to Jeff Rowberg for the I2CDev and MPU6050 libraries, and Richard Hirst for ServoBlaster. I made minor modifications to the I2CDev library for it to work on the Pi instead of an Arduino.


Copyright (c) 2018 Aitor Ormazabal

Licensed under the terms of the GNU General Public License, either version 3 or (at your option) any later version. A full copy of the license can be found in LICENSE.txt.