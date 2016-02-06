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

* ~show = False [bool] -- show the result instead of publishing it.
* ~encoding = "bgr8" [str] -- video encoding used by bridge.

"""

import rospy

import cv
import cv2
import numpy as np
import glob

from base import BaseStreamHandler


class Show(BaseStreamHandler):
    def on_image(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # gray = cv2.resize(gray, None, fx=.5, fy=.5, interpolation=cv2.INTER_CUBIC)
        ret, corners = cv2.findCirclesGrid(gray, (4, 11), None, cv2.CALIB_CB_ASYMMETRIC_GRID)

        if ret:
            for p in corners:
                y, x = p[0]
                img[x, y] = [0, 0, 255]

        return img


if __name__ == "__main__":
    Show.launch_node()
