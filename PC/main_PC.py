from MovementModule.movement import Movement
from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType
from ObjectDetectionModule.object_detection import ObjectDetection
from ObjectDetectionModule.od_handlers import ODHandlerType

import cv2

# === Delete all images from previous run ===
import os
import glob

files = glob.glob('Images/*.jpg')
for f in files:
    os.remove(f)
# ===================================

TURN_AMOUNT_MULTIPLIER = .8

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Initialize module objects
mv = Movement(calibration=10)
ld = LaneDetection(crop_range_h=(.85, .95), crop_range_w=(0, 1), method=LaneDetectionHandlerType.HYBRID)
od = ObjectDetection(handler_types={ODHandlerType.PEDESTRIAN, ODHandlerType.VEHICLE, ODHandlerType.TRAFFIC_LIGHT})

count = 0

while True:
    ret, img = cap.read()

    if not ret:
        print("cam not found")
        continue

    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow('original', img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    print(turn_amount)
    cv2.imshow("canny", canny)

    cv2.imwrite("Images/" + str(count) + " - t_a = " + str(turn_amount) + ".jpg", canny)
    count += 1

    if turn_amount:
        turn_amount = min(40, max(-40, turn_amount))
        mv.set_turn_amount(int(turn_amount * TURN_AMOUNT_MULTIPLIER))

    # Object Detection portion
    multiplier, det_img = od.get_speed_multiplier(img, True)
    mv.set_speed(100 * multiplier)

    mv.apply_speeds()
    cv2.imshow("detected", det_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

mv.reset_speeds()
cap.release()
cv2.destroyAllWindows()
