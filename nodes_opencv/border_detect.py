#!/usr/bin/env python
#
# OpenCV functions test: border detection
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""
This node is based on `base.py`. See there a documentation.


Inputs
------

* in/image -- main picture stream.


Outputs
-------

* out/image -- result image.


Parameters
----------

* ~method [str] -- border detection method to use.
* ~show = False [bool] -- show the result instead of publishing it.
* ~encoding = "bgr8" [str] -- video encoding used by bridge.

"""

import rospy

import cv
import cv2

from base import BaseStreamHandler


class Show(BaseStreamHandler):
    def __init__(self, method, **kwargs):
        self.method = method
        super(Show, self).__init__(**kwargs)

    def on_image(self, img):
        if self.method == 'laplacian':
            return self.laplacian(img)
        if self.method == 'sobel_x':
            return self.sobel_x(img)
        if self.method == 'sobel_y':
            return self.sobel_y(img)
        if self.method == 'canny':
            return self.canny(img)
        return img

    def laplacian(self, img):
        return cv2.Laplacian(img, cv2.CV_64F)

    def sobel_x(self, img):
        return cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=5)

    def sobel_y(self, img):
        return cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=5)

    def canny(self, img):
        return cv2.Canny(img, 100, 200)


if __name__ == "__main__":
    Show.launch_node(method='~method')
