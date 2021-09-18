from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType

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
ld = LaneDetection(crop_range_h=(.7, 1), crop_range_w=(0, 1), method=LaneDetectionHandlerType.MANY_ROWS)

# Loop over incoming camera frames
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    cv2.imshow("original", img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    print(turn_amount)
    cv2.imshow("canny", canny)
    
    rawCapture.truncate(0)

    if cv2.waitKey(1) == ord('q'):
        break

camera.close()
cv2.destroyAllWindows()

