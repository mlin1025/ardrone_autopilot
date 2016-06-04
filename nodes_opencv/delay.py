#!/usr/bin/env python
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
import cv2
import numpy as np
from base import BaseStreamHandler
from datetime import datetime


class DelayMesure(BaseStreamHandler):
    def __init__(self, *args, **kwargs):
        self.seek = False
        self.time = None

        self.black = np.zeros((600, 800, 3), np.uint8)
        self.white = np.zeros((600, 800, 3), np.uint8)
        self.white[:, :] = (255, 255, 255)

        rospy.Timer(rospy.Duration(1), self.toggle, oneshot=False)

        super(DelayMesure, self).__init__(*args, **kwargs)

    def toggle(self, *args, **kwargs):
        if not self.seek:
            cv2.imshow('Test', self.white)
            cv2.cv.WaitKey(1)
            self.seek = True
            self.time = datetime.now()

    def on_image(self, img):
        if self.seek and img.mean() > 100:
            print(datetime.now() - self.time)
            self.seek = False
            cv2.imshow('Test', self.black)
            cv2.cv.WaitKey(1)

        return img


if __name__ == "__main__":
    DelayMesure.launch_node()
