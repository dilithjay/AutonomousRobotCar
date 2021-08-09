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
ld = LaneDetection()

# Loop over incoming camera frames
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    cv2.imshow("original", img)

    l_speed, r_speed = ld.get_wheel_speeds(img)
    mv.set_speed(l_speed, r_speed)

    rawCapture.truncate(0)

    if cv2.waitKey(1) == ord('q'):
        break

camera.close()
cv2.destroyAllWindows()
