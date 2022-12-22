Machine Des Jeux: Robotic Arm
====================

This project contains code for a Arduino Uno/Mega used to receive serial command and control
3 stepper motors + 3 servo motors for a total of 6 axis.

It also contains a python script to be run by a machine (typically a RPi, Jetson or Windows PC for debugging) to send serial commands to the arduino with the custom protocol.

The python script may be adapted with a kinetics/pysics model of a robotic arm. For example, a model may process a 3d robot hand position into 6 angles which can be sent to the robotic arm by serial.

QuickStart Guide
----------------

1. The Arduino code (.ino file) shall be opened with Arduino IDE and the dependency libraries shall be installed. They should be available form the IDE itself, else google is your friend.
2. The Arduino code can then be flashed into the arduino with the required connections (see code) for axis 1 to 6.
3. The python script may be run from any python capable machine which is connected to the serial interface of the Arduino.
4. The only python dependency of the script pyserial. Google is your friend to know how to install it.
5. Modify the python script to your needs to send movement or configuration commands to the Arduino.