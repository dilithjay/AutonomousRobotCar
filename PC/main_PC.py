from MovementModule.movement import Movement
from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType
import cv2

# === Delete all images from previous run ===
import os
import glob

files = glob.glob('Images/*.jpg')
for f in files:
    os.remove(f)
# ===================================

TURN_AMOUNT_MULTIPLIER = 1.2

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Initialize module objects
mv = Movement()
ld = LaneDetection(crop_range_h=(.75, 1), crop_range_w=(0, 1), method=LaneDetectionHandlerType.MANY_ROWS)
count = 0

while True:
    ret, img = cap.read()

    if not ret:
        print("cam not found")
        continue

    cv2.imshow('original', img)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    # print(turn_amount)
    cv2.imshow("canny", canny)

    cv2.imwrite("Images/" + str(count) + " - t_a = " + str(turn_amount) + ".jpg", canny)
    count += 1

    if turn_amount:
        mv.set_turn_amount(turn_amount)

    # Object Detection portion (temp)
    mv.set_speed(100)

    mv.apply_speeds()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

mv.reset_speeds()
cap.release()
cv2.destroyAllWindows()
