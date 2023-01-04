from MovementModule.movement import Movement
from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType
from ObjectDetectionModule.object_detection import ObjectDetection

import cv2
import threading

# === Delete all images from previous run ===
import os
import glob

files = glob.glob('ObjectDetectionModule/images/*.jpg')
for f in files:
    os.remove(f)
# ===================================

TURN_AMOUNT_MULTIPLIER = 0.8

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Initialize module objects
mv = Movement(calibration=0)
ld = LaneDetection(crop_range_h=(.85, .95), crop_range_w=(0, 1), method=LaneDetectionHandlerType.HYBRID)
od = ObjectDetection()

count = 0
canny = None


def detect_lanes_and_apply_speed():
    global canny
    turn_amount, canny = ld.get_turn_amount(img)
    if turn_amount:
        turn_amount = min(40, max(-40, turn_amount))
        print(turn_amount)
        mv.set_turn_amount(int(turn_amount * TURN_AMOUNT_MULTIPLIER))
    mv.apply_speeds()
    cv2.imwrite(f"ObjectDetectionModule/images/{count}_{turn_amount}.jpg", canny)


while True:
    ret, img = cap.read()

    if not ret:
        print("cam not found")
        continue

    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow('original', img)
    # cv2.imwrite("ObjectDetectionModule/images/" + str(count) + "_original.jpg", img)

    # Lane Detection portion
    lane_det_thread = threading.Thread(target=detect_lanes_and_apply_speed)
    lane_det_thread.start()
    if canny is not None:
        cv2.imshow("canny", canny)

    # Object Detection portion
    multiplier, det_img = od.get_speed_multiplier(img, True)
    mv.set_speed(105)
    mv.set_speed_multiplier(multiplier)
    mv.apply_speeds()

    cv2.imshow("detections", det_img)
    cv2.imwrite(f"ObjectDetectionModule/images/det_{count}.jpg", det_img)
    count += 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

mv.reset_speeds()
cap.release()
cv2.destroyAllWindows()
