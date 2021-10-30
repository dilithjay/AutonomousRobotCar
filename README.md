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
* Added a script for testing on the pc and another script for testing on the Raspberry Pi with the movement module disabled.

_Note: The lane detection module is taking longer than I anticipated (at least to make it work on all situations encountered on the track). I have decided to focus entirely on getting this module to work before moving on to the rest of the modules, because lane detection is a fairly important portion of the robot. I intend to try other algorithms such as other variations of the main algorithm and in the case that that doesn't work, an end-to-end deep learning approach combined with a bit of preprocessing to identify lane lines._

### Week 7 & Week 8 (September 6th to September 19th)
* Changed the position of the camera to the back of the robot. This is because, as mentioned previously, the robot is seeing too far ahead and thus, the delay doesn't help in the case of sharp bends because the robot would be acting on something that was seen some time ago. The new camera position is placed at a much larger height such that there's a good view of the road immediately in fron of the car while also having a decent view ahead.
* Replaced one of the 3 ultrasonic sensors because it malfunctioned.
* Developed a controller to drive the robot using arrow keys. Shall be using this to collect data in case the deep learning approach is necessary for the lane detection.
* Noticed that the two wheels of the robot are moving at different speeds even when given the same PWM. It is mandatory that this isn't the case since the robot must move straight when the lane lines are at the correct angle. Ordered a new car chassis with four wheels in hopes that it would be more robust to differences in motor speeds. Additionally, the new car chassis has two layers of platforms which is great for additional space. Having four wheels should also better support the weight of the power bank which was previously an issue.
* I have combined Week 7 and 8 reports since there wasn't much progress due to exams and the unexpected problems. Hoping to catchup this week.

### Week 9 (September 20th to September 26th)
* Received the new four wheeled chassis.
* Unfortunately, the Raspberry Pi malfunctioned. It doesn't boot up when provided power. Several debugging methods were suggested in https://www.raspberrypi.org/forums/viewtopic.php?t=58151. As per the link, the main indicator that there exists an issue is that the ACT LED doesn't blink upon powering. The next thing to try was to plug the power without the SD card. The ACT LED still did not blink. According to the link, this indicates an issue with the EEPROM. The solution they specified was to take a separate SD card and store the bootloader on it. Providing power after inserting this SD card should ideally cause the ACT LED to blink rapidly. However, this does not seem to occur either. The link suggests that if that is the case, the Pi is likely irrecoverable. My guess is that I burnt something important with static although I don't recall anything out of the ordinary happening. There does not seem to be any external damage to the Raspberry Pi.

### Week 10 & Week 11 (September 27th to October 10th)
* The last two weeks were quite hectic in terms of progress.
* After looking around to find what alternatives I have to replace the Raspberry Pi, I found a method to use my mobile phone to get the camera input. Using an app called Iruin Webcam, I transmit the camera images from my phone to my laptop over Wi-fi. The received image is then processed and the wheel speeds are transmitted back to the Arduino over Bluetooth. At the Arduino, the rest of the behavior is same as the previous versions. Even though this setup has the disadvantage of network latency affecting the robot, it also has a few advantages such as:
  * More processing power on laptop compared to Raspberry Pi.
  * Removing the necessity of the power bank (both the Arduino and the motors can be powererd using the batteries).
* The camera would be placed on a raised tripod at angle and height such that the lane lines are visible.
* I started building the object detection models for vehicle detection and traffic light detection. I only collected a little data (around 50 images per class) just to get a base model working. The current dataset was captured under night time lighting so the remaining data needs to contain images captured with day time lighting.
* Thus the past too weeks were spent mostly on building the new setup.
* As expected, there were more challenges. After moving all the hardware onto the four wheel robot, it turns out that the robot now refuses to turn despite getting the acurate wheel speeds. This is apparently because the friction between the robot wheels and the track was too high and the wheels were unable to skid in order to make turns. My next attempt would be to move the hardware back to the initial robot but replace the motors with those that came with the new chassis.

### Week 12 (October 11th to October 17th)
<img src="https://user-images.githubusercontent.com/54039395/137118534-b7c4896a-a8a0-4c87-8146-a9b66f2cefc2.jpg" width="50%"><img src="https://user-images.githubusercontent.com/54039395/137118549-389dfc0a-68f8-487e-a7cb-6fa5b40c0004.jpg" width="50%">

* Tested different motors to identify a pair of motors with approximately equal speeds. A lot of the pairs have at least slight differences in rotation speed. Due to this, I decided to select two motors with approximately equal speeds and attached them to the robot. However, when the rest of the hardware were moved onto the robot, the difference in speeds seemed to become more prominent. Thus, I decided to apply the voltages to the motors with a callibration offset between the two. This means that the right motor (which in my case is the slower one) is given a higher PWM voltage compared to the left motor. While this doesn't always result in a perfect line, it mitigates the issues for the most part. The reason why it isn't too big of a problem is because the lane detection algorithm automatically changes the speeds when it goes too far off the track. However,  this results in a slightly wobbly movement for the robot (See video below).

https://user-images.githubusercontent.com/54039395/137106984-d5e7ed52-92a6-490a-a9fa-9e6196f8adfd.mp4

* Tried using Hough Lines in a different way:
  * Initial method: Find lines of left and right halves separately. Didn't work well when the road was too curved (curves aren't detected by Hough Lines method).
  * New method: Use the gradient of all lines to determine the turn amount (`= Mean gradient of positive gradients - Mean gradient of negative gradients`). This performed better than the initial method but turned out to be too sensitive false positive line detections. Thus, the `MANY_ROWS` method continues to be the better algorithm for this.

### Week 13 (October 18th to October 24th)
* Ordered LM393 Infrared Speed Sensor modules (yet to receive).
* Started working on the object detection module with the new system. Collected 65 images taken with objects at random locations, most of which have objects of 3 classes (4 classes in total: pedestrian, vehicle, red traffic light, green traffic light). The amount of data won't be sufficient for the final model but it's a sufficient baseline to work upwards from.
![image](https://user-images.githubusercontent.com/54039395/138223129-9bca9ab6-b9e5-47c0-bcd1-fd5e19118f96.png)
* Trained a model with the `ssd_resnet50_v1_fpn` model architecture using the Tensorflow Object Detection API. This uses the pretrained model `ssd_resnet50_v1_fpn_640x640_coco17` which is trained on the COCO dataset. A concept known as transfer learning is used where weights of an already trained network are used to initialize the new network. This results in much faster training with much less data. The configuration for the model were as follows:
  * **Batch size**: 8
  * **Data augmentation**: Random Cropping, Random Horizontal Flip
  * **Optimizer**: Gradient Descent with Momentum
* After training, it was noticed that the object detector had trouble with detecting traffic lights. Possible reason may include:
  * The base of the traffic light is black while the background of the track is also black.
  * The light from the LEDs are overexposed, resulting in them to look white, regardless of color.
  * Since only the enabled light was labelled, it may have been more difficult for the model.
* As a solution, a paper was placed on the traffic lights to prevent excess light.
* Vehicles and pedestrians were detected but these weren't consistent. The detector was highly biased towards pedestrians and was highly sensitive to light as well (this too is expected considering the lack of data).

![detection_1](https://user-images.githubusercontent.com/54039395/138226316-3d5bf640-7105-4158-a3e5-f52724b63d02.JPG)
* Implemented the logic for the object detection handlers:
  * For **Pedestrians and Vehicles**: The speed was multiplied by a value inversely proportional to the distance from 70% of the image. Any position higher than 70% of the image would stop the robot. The threshold of 70% is an arbitrary number that could be tuned during testing.
  * For **Traffic Lights**: If a traffic light was detected below 50% of the image, the robot would stop and begin looking for a green light in the bottom region.

### Week 14 (October 25th to October 31st)
* Trained a new model with the `efficientdet_d0` model architecture in the same method as previously mentioned. Configuration is as follows:
  * **Batch size**: 16
  * **Data augmentation**: Random Cropping, Random Horizontal Flip
  * **Optimizer**: Adam Optimizer
* The performance of this model was quite good. The model was able to consistently detect vehicles and pedestrians with 100% confidence. While there are a few false positives, these can be easily filtered out using a class specific threshold. However, the model seemed to lack confidence about traffic light detections. Despite this, the model does seem to make the detections consistently. Thus, similar to vehicles and pedestrians, a class-specific threshold would assist in filtering out the false positives.
* Started working integrating the speed encoders into the hardware.
  * It was noticed that `pin 10`, which was used as the `ENABLE A` line for the L298N motor driver, didn't work with PWM even though it was specified as one. Turns out that this is caused by the `TimerOne library` which was used to create interrupts at regular periods. Simply switching with `pin 11` (which was intially connected to an ultrasonic sensor) of the arduino fixed the issue.
