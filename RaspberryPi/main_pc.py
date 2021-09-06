from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType

# For Camera Input
import cv2

# Set up camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Initialize module objects
# mv = Movement()
ld = LaneDetection(crop_range_h=(.3, .9), crop_range_w=(0, 1), method=LaneDetectionHandlerType.MANY_ROWS)

# Loop over incoming camera frames
while True:
    ret, img = cap.read()
    cv2.imshow("original", img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    cv2.imshow("canny", canny)

    # TODO: Adjust delay according to speed
    # mv.set_delayed_turn_amount(.5, turn_amount)

    # Object Detection portion (temp)
    # mv.set_speed(100)

    # mv.apply_speeds()

    if cv2.waitKey(1) == ord('q'):
        break

# mv.reset_speeds()
cap.release()
cv2.destroyAllWindows()
