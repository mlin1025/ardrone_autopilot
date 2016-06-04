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
import tf
from tf.transformations import quaternion_matrix

import numpy as np

import image_geometry

import math

from base import BaseStreamHandler


class Show(BaseStreamHandler):
    def __init__(self, *args, **kwargs):
        self.tf = tf.TransformListener()
        self.camera_model = image_geometry.PinholeCameraModel()

        super(Show, self).__init__(*args, **kwargs)

    def on_image(self, img):
        if self.info is None:
            return

        self.camera_model.fromCameraInfo(self.info)
        # self.camera_model.rectifyImage(img, img)

        self.tf.waitForTransform('ardrone/odom',
                                 'ardrone/ardrone_base_frontcam',
                                 rospy.Time(0),
                                 rospy.Duration(3))

        trans, rot = self.tf.lookupTransform('ardrone/odom',
                                             'ardrone/ardrone_base_frontcam',
                                             rospy.Time(0))

        rot_matrix = np.array(quaternion_matrix(rot))

        for a in range(0, 360, 30):
            vector = np.array(np.array([0.1 * math.cos(a * math.pi / 180), 0.1 * math.sin(a * math.pi / 180), 0, 0]))
            point = vector.dot(rot_matrix)
            x, y = self.camera_model.project3dToPixel(point)
            cv2.circle(img, (int(x), int(y)), 5, (0, 0, 255), -1)

        return img

if __name__ == "__main__":
    Show.launch_node()
