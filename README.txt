A Quadcopter flight controller for the Raspberry Pi.
The program supposes that a ServoBlaster daemon is running and it writes PWM outputs to /dev/servoblaster. You can edit Servo::writeMicroseconds to write the outputs in some other way.

Input is recieved through a UNIX socket created by the program at startup, in the form of 4 byte udp packets. I use a separate python program to recieve commands from an Android app and send them to the flight controller through the socket. You can edit Input::Update() to change how input is recieved.



Credit to Jeff Rowberg for the I2CDev and MPU6050 libraries, and Richard Hirst for ServoBlaster. I made minor modifications to the I2CDev library for it to work on the Pi instead of an Arduino.
