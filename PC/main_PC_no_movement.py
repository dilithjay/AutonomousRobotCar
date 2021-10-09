from MovementModule.movement import Movement
from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType
import cv2


# Initialize module objects
mv = Movement()
ld = LaneDetection(crop_range_h=(.7, 1), crop_range_w=(0, 1), method=LaneDetectionHandlerType.MANY_ROWS)

cap = cv2.VideoCapture(1)
# count = 0

while True:
    ret, img = cap.read()
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow('original', img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    print(turn_amount)

    cv2.imshow("canny", canny)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

mv.reset_speeds()
cap.release()
cv2.destroyAllWindows()
