import cv2
import numpy as np


def get_canny(image):
    """
    Get the image with the canny edge detector applied.

    :param image: Image on which edges need to be detected.
    :return: Image with just the detected edges.
    """
    gray = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 8)
    canny = cv2.Canny(blur, 50, 150)
    return canny


cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

count = 40

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    image = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = get_canny(image)
    # h, w = frame.shape[:2]
    # frame = frame[int(h * 0.75):, :]

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Display the resulting frame
    cv2.imshow('canny', frame)
    cv2.imshow('original', image)
    # sleep(0.5)
    if cv2.waitKey(1) == ord('c'):
        cv2.imwrite('Images/' + str(count) + '.jpg', frame)
        print("Image saved:", str(count) + '.jpg')
        count += 1
    if cv2.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()