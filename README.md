# Autonomous Robot Car
A robot car capable of driving autonomously on a track containing obstacles such as pedestrians, other vehicles and traffic lights.

## Weekly Reports
### July 26th to August 1st
* Created the basic setup for communication between Arduino and Raspberry Pi using I2C.
* Added functionality to control the motors from the Raspberry Pi via the Arduino and motor driver.
* Created simulation on Webots to test a simple lane detection algorithm where the bottom half of the image is used to detect edges towards the left and right of the camera views and move the robot in an attempt to center it between the edges.
