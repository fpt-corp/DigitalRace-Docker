import cv2
import numpy as np

import find
import classification

SIGN_NONE = 0
SIGN_TURN_RIGHT = 1
SIGN_TURN_LEFT = 2


class SignDetector:
    def __init__(self, path):
        self.object_finder = find.ObjectFinder()
        self.sign_classifier = classification.CNN(path)

    def detect_sign(self, img):
        """
        Detect and classify a traffic sign in the input image.

        :param img: numpy RGB image, not normalized (range 0-255)
        :return: 2 values, including:
            ___1. int sign type:
                0: SIGN_NONE
                1: SIGN_TURN_RIGHT
                2: SIGN_TURN_LEFT
            ___2. a tuple specifying sign position and size (x, y, w, h), in which (x,y) is the coordinate of the
            top left pixel of the traffic sign, w is width and h is height.

        Examples
        --------
        >>> detect_sign(rgb_frame)
        1, (60, 60, 100, 100)

        Warning
        ------
        Input image with a different colour space from RGB might give inaccurate result.
        """

        sign_imgs, points = self.object_finder.find_by_color(img)

        for index in range(len(sign_imgs)):
            prediction = self.sign_classifier.predict_sign(sign_imgs[index])
            if prediction != 0:
                return prediction, points[index]

        return 0, (0, 0, 0, 0)
