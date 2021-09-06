# Autonomous Robot Car
The goal of this project is to develop a robot car capable of navigating autonomously on a down-scaled track containing obstacles such as pedestrians, other vehicles, etc.. The robot is also expected to stay within the the bounds of lane lines while abiding road rules such as traffic lights. Additionally, the robot shall have a fail-safe such that in the case that the remaining modules fail, the robot should be capable of avoiding collisions.

The hardware components being used for this robot and their prices are as follows:
| Component Name | Price |
| -------------- | ----: |
| Raspberry Pi 4 Model B | LKR 10,000 |
| Pi Camera Module v2 | LKR 6,000 |
| Arduino UNO | LKR 1,000 |
| L298N Motor Driver | LKR 400 |
| HC-SR04 Ultrasonic Sensors x3 | LKR 900 |
| Robot Car Chassis + Gear Motors + Wheels | LKR 1,000 |
| 20000mAh Xiaomi Power Bank | LKR 7,000 |
| TR 14500 Li-ion batteries x2 | LKR 700 |

| Total Price | LKR 27,000 |
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

### Week 3 (August 9th to August 15th)
* Tested ultrasonic sensors and selected 3 with minimal glitches (a few sensors gave incorrect readings intermittently).
* Built a frame to hold all the hardware and finished assembling them. This will likely be the setup I'll use to test for the entirety of the project.
* A couple of challenges faced during assembly:
  * Limited space for placing all hardware components. Solution: Built a 2nd platform to place the motor driver and batteries.
  * The Field of View of the camera was noticed to be quite low. As a result, the path was largely not within view. Solution: Added a slanted bar to look have the camera point at the front of the car.
* Implemented fail safe on Arduino. Not yet tested.

<img src="https://user-images.githubusercontent.com/54039395/129077279-8ed3d4fd-fd20-4624-a787-7a1bc6ab9a12.jpg" width="50%"><img src="https://user-images.githubusercontent.com/54039395/129077318-a8a559f9-0b6b-4979-ad87-a4a1e27da410.jpg" width="50%">
<p align="center"><i>Robot Car</i></p>
<p align="center">
  <img src="https://user-images.githubusercontent.com/54039395/129077575-9851f956-6d24-4f6e-aa3a-170f794f4eda.jpg" width="70%"><br>
  <i>Track</i>
</p>

### Week 4 (August 16th to August 22nd)
* Made calculations for angle of the Pi Camera (based on Field of View / FOV and height) and adjusted the camera on the car, accordingly:

<img src="https://user-images.githubusercontent.com/54039395/129904842-99c70a72-85ae-4b3a-b10d-7c4673c639fa.jpg" width="49%"> <img src="https://user-images.githubusercontent.com/54039395/129913353-035cf7bf-c606-477a-ae2c-3f6b25ae1b9a.jpg" width="49%">

Even though the width of the road between lane lines is approximately 20 cm, calculations were made using 25 cm to leave room for error. The horizontal FOV of the Pi camera module v2 is `62.2 degrees` while the vertical FOV is `48.8 degrees`. A triangle can be formed using this width and the boundaries of the FOV (see 1st diagram). Using this triangle and the angles of each corner, the perpendicular distance to the width is calculated as `width / 2 * tan((180 - 62.2) / 2) = 20.72 cm`

Using the 2nd diagram, a perpendicular line can be drawn from the point of contact of the lower bound of the FOV to the camera holding bar. The length of this perpendicular can be calculated as `20.72 * sin(90 - 48.8/2) = 18.87 cm`. The distances to the incidence point of this perpendicular on the bar from the base of the bar is equal to `9.5 - 20.72 * cos(90 - 48.8/2) = 0.94 cm`. Therefore, an equation for the angle (say `x`) of the bar can be obtained as `0.94 * sin(x) + 9.5 = 18.87 * cos(x)`. Solving this equation, we get `x = 57 degrees`. Thus, the tilt of the bar was adjusted accordingly.

### Week 5 (August 23rd to August 29th)
* Tested the fail-safe module. It is successful at avoiding collisions. Further tuning of parameters such as distance check frequency and thresholds is necessary.
* Refactored the movement module such that it accepts the average `speed` and a `turn amount` (a measure of how much to the right the robot should turn). These metrics were used to calculate the left and right wheel speeds separately. This method is preferred because the lane detection module would only control the `turn amount` while the object detection modules would only control the `speed`. This makes it easier to calculate the minimum speed since both wheel speeds need not be considered separately.
* Tested the lane detection module. The lower bound of the camera view was too far from the base of the car (approx. `15 cm` away). Accordingly, shortened the length of the bar holding the camera to `6 cm` which led to the lower bound of the view coming up to `9 cm` away from the base.
* Used threading to delay the `turn amount` affecting the wheel speeds. This is added to further mitigate the problem of the base being too far ahead since the action required for the current view should be executed once the robot has arrived at that position. Currently, the delay is set to one second. However, it needs to be adjusted at runtime based on the speeds of the robot.
* Concerns: Currently, the robot seems to be moving at quite a low speed. This is likely due to how heavy the robot is (approx. `1 kg`). The power bank is the heaviest component of the robot (approx `360 g`). A lower capacity power bank would likely weigh much less. 20,000 mAh is somewhat overkill and thus, can be reduced. Additionally, further weight can be dropped by switching to a 3D printed frame (which holds the components) since it can be designed more efficiently (as opposed to using hardboard boxes).

### Week 6 (August 30th to September 5th)
* Began testing the lane detection module with the altered camera orientation and position. The robot seems to be following the road consistently when both lanes are within vision. However, when one of the lane lines move out of vision, the robot's trajectory is often incorrect.
 * Developed a second lane detection algorithm that predicts a straight line onto the lanes. The gradient of both the straight lines was compared against a constant value (set to `2` after determining using a stationary frame), the result of which is used to calculate the turn amount. Even though the results were satisfactory in general, the algorithm failed to use the correct line in the case of sharp bends of approx. `90 degrees`.
* Added a script for testing on the pc and another script for testing on the Raspberry Pi without the movement module disabled.

_Note: The lane detection module is taking longer than I anticipated (at least to make it work on all situations encountered on the track). I have decided to focus entirely on getting this module to work before moving on to the rest of the modules, because lane detection is a fairly important portion of the robot. I intend to try other algorithms such as other variations of the main algorithm and in the case that that doesn't work, an end-to-end deep learning approach combined with a bit of preprocessing to identify lane lines._
