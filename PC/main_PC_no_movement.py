from LaneDetectionModule.lane_detection import LaneDetection, LaneDetectionHandlerType
from ObjectDetectionModule.object_detection import ObjectDetection
from ObjectDetectionModule.od_handlers import ODHandlerType
import cv2


# Initialize module objects
ld = LaneDetection(crop_range_h=(.85, .95), crop_range_w=(0, 1), method=LaneDetectionHandlerType.HYBRID)
# od = ObjectDetection(handler_types={ODHandlerType.PEDESTRIAN, ODHandlerType.VEHICLE, ODHandlerType.TRAFFIC_LIGHT})

cap = cv2.VideoCapture(1)
# count = 0

while True:
    ret, img = cap.read()
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow("original", img)

    # Lane Detection portion
    turn_amount, canny = ld.get_turn_amount(img)
    print("Turn amount:", turn_amount)
    # multiplier, det_img = od.get_speed_multiplier(img, True)
    # print("Multiplier:", multiplier)

    # cv2.imshow('detection', det_img)
    cv2.imshow("canny", canny)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
