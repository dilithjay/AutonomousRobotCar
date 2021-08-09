# Autonomous Robot Car
The goal of this project is to develop a robot car capable of navigating autonomously on a down-scaled track containing obstacles such as pedestrians, other vehicles, etc.. The robot is also expected to stay within the the bounds of lane lines while abiding road rules such as traffic lights. Additionally, the robot shall have a fail-safe such that in the case that the remaining modules fail, the robot should be capable of avoiding collisions.

The hardware components being used for this robot and their prices are as follows:
| Component Name | Price |
| -------------- | ----- |
| Raspberry Pi 4 Model B | LKR 10,000 |
| Pi Camera Module v2 | LKR 6,000 |
| Arduino UNO | LKR 1,000 |
| L298N Motor Driver | LKR 400 |
| HC-SR04 Ultrasonic Sensors x3 | LKR 900 |
| Robot Car Chassis + Gear Motors + Wheels | LKR 1,000 |
| 20000mAh Xiaomi Power Bank | LKR 7,000 |
| TR 14500 Li-ion batteries x2 | LKR 700 |

| Total Price | LKR 20,000 |
| ------- | ----- |

![image](https://user-images.githubusercontent.com/54039395/128195441-8c4cc0fa-7e37-4d53-9361-a5647f88b1a4.png)

Accordingly, the following subtasks need to be addressed:
* Movement: Rotate the wheels at the speeds decided by the Raspberry Pi and Arduino.
* Fail-safe: Stop or slow down vehicle if object gets closer than a threshold.
* Lane detection: Detect the path and decide on wheel speeds accordingly.
* Object Detection: Pedestrian/vehicle/traffic light detection.

![image](https://user-images.githubusercontent.com/54039395/128195567-983ded8c-820f-4d16-b21a-72ef03cb2f43.png)

## Project Plan
Note that the following plan is fairly optimistic since there are bound to be delays which is why I have left several weeks towards the end to account for delays and provide time for further testing.
| Week | Task |
|:---:| ------ |
| 1 | *	Create GitHub repo<br>*	Setup Arduino-Raspberry Pi communication<br>*	Build simulation on Webots |
| 2 | *	Assemble components required for movement.<br>*	Implement movement module.<br>*	Build base track<br>*	Begin implementing lane detection module. |
| 3 | *	Complete implementation of lane detection.<br>*	Test techniques on track.<br>*	Begin building fail-safe module |
| 4 | * Build traffic light device and collect images for training.<br>*	Choose object for pedestrian and collect images for training. |
| 5 | * Choose an object to represent the vehicle and collect images.<br>*	Begin looking into possible methods of training a custom object detector. |
| 6 | *	Train model and optimize. |
| 7 | *	Setup track.<br>*	Test vehicle on track. |
| Remaining<br>weeks | Reserved for further testing (integration testing) and/or to catch up on any delays in prior weeks. |

## Weekly Reports
### Week 1 (July 26th to August 1st)
* Created the basic setup for communication between Arduino and Raspberry Pi using I2C.
* Added functionality to control the motors from the Raspberry Pi via the Arduino and motor driver.
* It was noticed that at low frequencies of PWM provided by the Arduino, the motor tends to not rotate without a jumpstart. This is likely due to the friction of the gears in the motors and some machine oil might help.
* Created simulation on Webots to test a simple lane detection algorithm where the bottom half of the image is used to detect edges towards the left and right of the camera views and move the robot in an attempt to center it between the edges.

https://user-images.githubusercontent.com/54039395/128219606-5386d9f3-06b2-4d6c-a850-e92531aedd91.mp4

### Week 2 (August 2nd to August 8th)
* Ordered some rechargeable batteries (model: 18650) to increase the power provided to the motors. This would likely help since currently it's been powered by USB 5V but the motors and motor driver require a combined total of 7.4V to operate at maximum power. (Received TR 14500 batteries instead of 18650 which is fine.)
* Created a project plan. 
* Created a temporary frame to place the components and power bank.
* Implemented the movement module in Python.
*	Built part of the track to help testing.
*	Began implementing lane detection module in Python.

#### Demo (as of August 4th)
The following is a demo of the implementation so far. I provide the required speeds (range 0 to 255 / 1 byte) to a python program on the Raspberry Pi which gives commands to the Arduino. The Arduino outputs the speeds into the motor driver which in turn controls the motors.

As seen in the video, the speeds are given as input in the format `{left speed} {right speed}`. Setting either of them to zero stops the respective wheel(s). Setting either to a value less than 255 would slow down the wheel(s).

https://user-images.githubusercontent.com/54039395/128216670-790ba952-6d19-4c42-b52d-080d40fb329c.mp4
