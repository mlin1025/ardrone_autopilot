#!/usr/bin/env python
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""Target recognition and visualization


Inputs
------

* /in/image -- main picture stream.
* /in/info -- camera info stream.


Outputs
-------

* /out/image -- result image.


Parameters
----------

* ~encoding = "bgr8" [str] -- video encoding used by bridge.
* ~path = "../data/target.jpg" [str] -- path to the target sample
  (relative to the file location).

"""

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import rospy

from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, CameraInfo

from os.path import dirname, join
from operator import attrgetter, is_not
from collections import deque
from itertools import takewhile
from functools import partial

import tf
from tf.transformations import euler_from_quaternion, euler_matrix

import image_geometry

from math import cos, sin

from utils.drone import DroneController

from datetime import datetime, timedelta

# _x__ = datetime.now()

# def t_in():
#     global _x__
#     _x__ = datetime.now()

# def t_out():
#     global _x__
#     print datetime.now() - _x__,



class MovingAverage(object):
    def __init__(self, coeffs=None):
        if coeffs is None:
            coeffs = [1.0] * 5

        self.coeffs = coeffs
        self.items = deque(maxlen=len(coeffs))

    def __call__(self, new_val):
        self.items.appendleft(new_val)
        items = list(takewhile(partial(is_not, None), self.items))
        if items:
            return sum(c * v for c, v in zip(self.coeffs, items)) / len(items)


class Detect(object):
    def __init__(self, kp=None, desc=None):
        """Keypoints and descriptors of an image"""
        self.kp = kp
        self.desc = desc


class ControllerNode(object):
    def __init__(self):
        self._b = False
        self.last_cmd_sent = datetime.now()
        self.controller = DroneController()
        self.min_points = 15
        self.filter = MovingAverage()
        self.tf = tf.TransformListener()
        self._ap_ctrl = rospy.Subscriber('/apctrl', Empty, self.on_toggle, queue_size=1)
        ####
        self.targetGetter = rospy.Subscriber('target', String, self.on_target, queue_size=5)
        ####
        self.is_active = False
        self.is_webcam = rospy.get_param('~is_webcam', False)

    def send_vel(self, x=0, y=0):
        if self.is_active:
            if x == 0 and y == 0:
                self.controller.hover()
            else:
                self.controller.send_vel(x, y)
            print(x, y, str(datetime.now() - self.last_cmd_sent))
            self.last_cmd_sent = datetime.now()

    def on_toggle(self, e):
        self.is_active = not self.is_active
        print(self.is_active)


###    def on_target(self, String str):
        

    def get_rot_matrix(self, rev=False):
        frm, to = 'ardrone/odom', 'ardrone/ardrone_base_bottomcam'
        if rev:
            frm, to = to, frm

        self.tf.waitForTransform(frm, to, rospy.Time(0), rospy.Duration(3))
        trans, rot = self.tf.lookupTransform(frm, to, rospy.Time(0))

        x, y, z = euler_from_quaternion(rot)
        yaw = euler_from_quaternion(rot, axes='szxy')[0]

        return yaw, np.array(euler_matrix(-x, -y, z))

    def process_data(self, local_coordinates):
        coordinates = self.filter(local_coordinates)
        if coordinates is None:
            if datetime.now() - self.last_cmd_sent > timedelta(seconds=0.3):
                self.send_vel(0, 0)
            return

        coordinates.resize(1, 4)

        yaw, rot_matrix = self.get_rot_matrix(rev=True)

        point = coordinates.dot(rot_matrix)

        if any(point[0]) and point[0][2] < 5000:
            x, y, z = point[0][0], point[0][1], point[0][2]
            x, y = x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw)

            y -= 50

            left = (min(max(x, -700), 700) / 10000)
            forward = -(min(max(y, -700), 700) / 10000)

            self.send_vel(forward, left)

        if datetime.now() - self.last_cmd_sent > timedelta(seconds=0.3):
            self.send_vel(0, 0)

    #         self.project(img, point[0])

    # def project(self, img, point):
    #     if self.info is None:
    #         return

    #     self.camera_model.fromCameraInfo(self.info)
    #     # self.camera_model.rectifyImage(img, img)

    #     _, rot_matrix = self.get_rot_matrix()

    #     point = point.dot(rot_matrix)

    #     x, y = self.camera_model.project3dToPixel(point)
    #     cv2.circle(img, (int(x), int(y)), 10, (0, 0, 255), 2)
    #     cv2.circle(img, (int(x), int(y)), 15, (0, 0, 255), 2)
    #     cv2.circle(img, (int(x), int(y)), 20, (0, 0, 255), 2)

    #     point = np.array([0, 0, -1, 0])
    #     point = point.dot(rot_matrix)
    #     x, y = self.camera_model.project3dToPixel(point)
    #     cv2.circle(img, (int(x), int(y)), 10, (0, 0, 255), 2)


if __name__ == "__main__":
    rospy.init_node('ui_node')

    rospy.loginfo('Starting autopilot node')

    node = ControllerNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
