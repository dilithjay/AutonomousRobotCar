import cv2

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

count = 40

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    # h, w = frame.shape[:2]
    # frame = frame[int(h * 0.75):, :]

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Display the resulting frame
    cv2.imshow('frame', frame)
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