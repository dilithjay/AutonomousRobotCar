from MovementModule.movement import Movement
from LaneDetectionModule.lane_detection import LaneDetection

# For Camera Input
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

# Set up camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Initialize module objects
mv = Movement()
ld = LaneDetection(crop_range_w=(0, 1))

# Loop over incoming camera frames
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    cv2.imshow("original", img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    cv2.imshow("canny", canny)
    if turn_amount is not None:
        mv.set_turn_amount(turn_amount)

    # Object Detection portion (temp)
    mv.set_speed(180)

    mv.apply_speeds()
    rawCapture.truncate(0)

    if cv2.waitKey(1) == ord('q'):
        break

mv.reset_speeds(0, 0)
camera.close()
cv2.destroyAllWindows()
