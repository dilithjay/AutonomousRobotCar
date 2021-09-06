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
rawCapture = PiRGBArray(camera, size=camera.resolution)

# Initialize module objects
mv = Movement()
ld = LaneDetection(crop_range_h=(.5, .9), crop_range_w=(0, 1))

# Loop over incoming camera frames
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    cv2.imshow("original", img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    cv2.imshow("canny", canny)

    # TODO: Adjust delay according to speed
    if turn_amount:
        mv.set_delayed_turn_amount(.5, turn_amount)

    # Object Detection portion (temp)
    mv.set_speed(100)

    mv.apply_speeds()
    rawCapture.truncate(0)

    if cv2.waitKey(1) == ord('q'):
        break

mv.reset_speeds()
camera.close()
cv2.destroyAllWindows()
