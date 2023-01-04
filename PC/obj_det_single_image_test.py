from ObjectDetectionModule.object_detection import ObjectDetection
import cv2

od = ObjectDetection()

img_path = "ObjectDetectionModule/images/10_original.jpg"
img = cv2.imread(img_path)

multiplier, det_img = od.get_speed_multiplier(img)
cv2.imshow("detections", det_img)
print(multiplier)

cv2.waitKey(0)
