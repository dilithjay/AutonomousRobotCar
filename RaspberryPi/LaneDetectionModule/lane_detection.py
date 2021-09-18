import cv2
import numpy as np
from enum import Enum

AVG_GRAD = 2


class LaneDetectionHandlerType(Enum):
    ONE_ROW = 1
    MANY_ROWS = 2
    LINE_PREDICT = 3


class LaneDetection:
    def __init__(self, calibration=0, crop_range_w=(0.2, 0.8), crop_range_h=(0.5, 1), blur_kernel_size=(3, 3),
                 canny_threshold_range=(30, 90), row_count=5, method=LaneDetectionHandlerType.ONE_ROW):
        """
        Initialize the Lane Detection module.

        :param calibration: Displacement of the center of the camera.
        :param crop_range_w: Fraction of the width to be considered when detecting lanes.
        :param crop_range_h: Fraction of the height to be considered when detecting lanes.
        :param blur_kernel_size: Size of the Gaussian Blur kernel
        :param canny_threshold_range: Low and high thresholds of Canny Edge Detector.
        :param row_count: Number of rows considered for MANY_ROWS
        :param method: Import LaneDetectionHandlerType enum to change how wheel speeds are calculated.
        """
        self.calibration = calibration
        self.crop_range_w = crop_range_w
        self.crop_range_h = crop_range_h
        self.kernel_size = blur_kernel_size
        self.low_threshold, self.high_threshold = canny_threshold_range
        self.row_count = 20
        self.method = method

        self.correct_fraction = 0.4

    def get_canny(self, image):
        """
        Get the image with the canny edge detector applied.

        :param image: Image on which edges need to be detected.
        :return: Image with just the detected edges.
        """
        gray = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.kernel_size, 8)
        canny = cv2.Canny(blur, self.low_threshold, self.high_threshold)
        return canny

    def set_automatic_threshold(self, image):
        """
        Method to set the Canny thresholds automatically.

        :param image: Image based on which thresholds are to be determined
        """
        image = image.astype("uint8")
        self.high_threshold, thresh_im = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.low_threshold = 0.5 * self.high_threshold

    def crop_image(self, image, lines=None):
        """
        Method to crop image. Crops horizontally by crop_range_w. Crops the specified number of lines
        vertically if `lines` is defined. Else crops by crop_range_h.

        :param image: Image to be cropped
        :param lines: Number of rows of pixels in cropped image
        :return: Cropped image, cropped image's height, cropped image's width
        """
        h, w = image.shape[:2]
        range_w = (int(self.crop_range_w[0] * w), int(self.crop_range_w[1] * w))
        if not lines:
            range_h = (int(self.crop_range_h[0] * h), int(self.crop_range_h[1] * h))
            return image[range_h[0]:range_h[1], range_w[0]:range_w[1]], range_h[1] - range_h[0], range_w[1] - range_w[0]
        return image[-lines:, range_w[0]:range_w[1]], lines, range_w[1] - range_w[0]

    def get_speed_fractions(self, image):
        """
        Method to get left and right speeds as fractions.

        :param image: Image to be used to make the decision.
        :return: Left wheel speed, Right wheel speed (range: 0 - 1; None if couldn't find edges), canny image.
        """

        cropped, h, w = self.crop_image(image)
        canny = self.get_canny(cropped)
        left, right = 0, 0

        if self.method == LaneDetectionHandlerType.ONE_ROW:
            left, right = self.get_speed_fractions_one_row(canny, w, h)
        elif self.method == LaneDetectionHandlerType.LINE_PREDICT:
            left, right = self.get_speed_fractions_line_predict(canny, w)
            # Temp: Added to see lines when debugging
            lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=50)
            canny = cv2.merge([canny, canny, canny])
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(canny, (x1, y1), (x2, y2), (0, 255, 0), thickness=2, lineType=8)
        elif self.method == LaneDetectionHandlerType.MANY_ROWS:
            for i in range(0, h, 15):
                l, r = self.get_speed_fractions_one_row(canny, w, h - i)
                left += l
                right += r
            left /= h/15
            right /= h/15

        return left, right, canny

    def get_speed_fractions_line_predict(self, canny, w):
        left, right = 0, 0
        left_img, right_img = canny[:, :w // 2 - self.calibration], canny[:, w // 2 - self.calibration:]
        left_lines = cv2.HoughLinesP(left_img, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=50)
        right_lines = cv2.HoughLinesP(right_img, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=50)

        if left_lines is not None:
            left = self.get_fraction_using_line_grads(left_lines)
        if right_lines is not None:
            right = self.get_fraction_using_line_grads(right_lines)
        return left, right

    @staticmethod
    def get_fraction_using_line_grads(lines):
        avg_grad = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            avg_grad += abs((y2 - y1) / (x2 - x1))
        avg_grad /= len(lines)
        fraction = 1 - avg_grad / AVG_GRAD
        return fraction

    def get_speed_fractions_one_row(self, canny, w, row):
        max_pixels = w // 2 - self.calibration
        left, right = max_pixels, max_pixels
        found_l, found_r = False, False
        for i in range(max_pixels):
            p1 = canny[row - 1, max_pixels + i]
            p2 = canny[row - 1, max_pixels - i]
            if not found_l and p1 > 0:
                found_l = True
                left = i
            if not found_r and p2 > 0:
                found_r = True
                right = i
            if found_l and found_r:
                break
        return left/max_pixels, right/max_pixels

    def get_turn_amount(self, image):
        """
        Calculate the turn amount (how much to turn right) using speed fractions
        :param image: Image to be used to make the decision.
        :return: turn amount (None if cannot find), canny image
        """
        l_fraction, r_fraction, canny = self.get_speed_fractions(image)

        if l_fraction == 0 and r_fraction > 0:
            turn_amount = int((self.correct_fraction - r_fraction) * 64)
        elif r_fraction == 0 and l_fraction > 0:
            turn_amount = int((l_fraction - self.correct_fraction) * 64)
        elif l_fraction == 0 and r_fraction == 0:
            turn_amount = None
        else:
            print(l_fraction, r_fraction, end=", ")
            turn_amount = int((l_fraction - r_fraction) * 64)
            if abs(turn_amount) < 2:
                self.correct_fraction = l_fraction
        print(turn_amount)

        return turn_amount, canny
