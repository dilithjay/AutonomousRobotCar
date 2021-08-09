import cv2
import numpy as np
from enum import Enum


class LaneDetectionHandlerType(Enum):
    LAST_ROW = 1


class LaneDetection:
    def __init__(self, calibration=0, crop_range_w=(0.2, 0.8), crop_range_h=(0.5, 1), blur_kernel_size=(3, 3),
                 canny_threshold_range=(30, 90), method=LaneDetectionHandlerType.LAST_ROW):
        """
        Initialize the Lane Detection module.

        :param calibration: Displacement of the center of the camera.
        :param crop_range_w: Fraction of the width to be considered when detecting lanes.
        :param crop_range_h: Fraction of the height to be considered when detecting lanes.
        :param blur_kernel_size: Size of the Gaussian Blur kernel
        :param canny_threshold_range: Low and high thresholds of Canny Edge Detector.
        :param method: Import LaneDetectionHandlerType enum to change how wheel speeds are calculated.
        """
        self.calibration = calibration
        self.crop_range_w = crop_range_w
        self.crop_range_h = crop_range_h
        self.kernel_size = blur_kernel_size
        self.low_threshold, self.high_threshold = canny_threshold_range
        self.method = method
        self.prev_k = 0

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

    def get_direction(self, image, canny=True, crop=True, lines=None):
        """
        Method to get a measure of how much to turn from the current direction.

        :param image: Image to use to make the decision.
        :param canny: Whether canny edge detector should be applied.
        :param crop: Whether image needs to be cropped.
        :param lines: Whether lines should be used if image should be cropped
        :return: Measure of how much to turn based on current direction.
        """
        if crop:
            image, h, w = self.crop_image(image, lines)
        else:
            h, w = image.shape[:2]
        if canny:
            image = self.get_canny(image)
        k1 = k2 = -1
        for i in range(w // 2):
            p1 = image[h - 1, w // 2 - self.calibration + i]
            p2 = image[h - 1, w // 2 - self.calibration - i]
            if k1 == -1 and p1 > 0:
                k1 = i
            if k2 == -1 and p2 > 0:
                k2 = i
            if k1 != -1 and k2 != -1:
                self.prev_k = k1 - k2
                break
        return self.prev_k

    def get_wheel_speeds(self, image):
        """
        Method to get left and right wheel speeds.

        :param image: Image to be used to make the decision.
        :return: Left wheel speed, Right wheel speed (range: 0 - 255). None if couldn't find edges.
        """

        cropped, h, w = self.crop_image(image)
        canny = self.get_canny(cropped)

        if self.method == LaneDetectionHandlerType.LAST_ROW:
            left = right = -1
            max_pixels = w // 2 - self.calibration
            for i in range(max_pixels):
                p1 = canny[h - 1, max_pixels + i]
                p2 = canny[h - 1, max_pixels - i]
                if left == -1 and p1 > 0:
                    left = i
                if right == -1 and p2 > 0:
                    right = i
                if left != -1 and right != -1:
                    return left / max_pixels, right / max_pixels
        return None, None
