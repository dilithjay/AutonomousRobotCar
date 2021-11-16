import cv2
import numpy as np
from lane_detection import LaneDetection, LaneDetectionHandlerType

ld = LaneDetection(crop_range_h=(0.85, .95), crop_range_w=(0, 1), method=LaneDetectionHandlerType.HYBRID)

img_path = "D:/AutonomousRobotCar/PC/LaneDetectionModule/images/5.jpg"
img = cv2.imread(img_path)
# cv2.imshow("original", img)

turn_amount, canny = ld.get_turn_amount(img)
lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 20, minLineLength=30, maxLineGap=3)
rgb = cv2.cvtColor(canny, cv2.COLOR_GRAY2RGB)
if lines is not None:
    for i in range(0, min(2, len(lines))):
        line = lines[i][0]
        print(line)
        cv2.line(rgb, line[:2], line[2:], (0, 0, 255), 3, cv2.LINE_AA)
    # print(len(lines))
print("Turn amount:", turn_amount)
cv2.imshow("canny", rgb)

cv2.waitKey(0)
