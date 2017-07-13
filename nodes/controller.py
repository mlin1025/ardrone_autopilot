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

from std_msgs.msg import Empty
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

        self.bridge = CvBridge()
        self.encoding = rospy.get_param('~encoding', 'bgr8')

        #self.info = None

        #rospy.Subscriber('/ardrone/localTargetCoordsWhichWeShouldProccessAndFollow', Vector3, self.process_data, queue_size=5)

        path = join(dirname(dirname(__file__)), 'data/target.jpg')
        path = rospy.get_param('~path', path)

        self.detector = cv2.ORB_create()
        self.pattern = cv2.imread(path, 0)
        self.pattern_detect = self.detect(self.pattern)

        self.min_points = 15

        self.matcher = cv2.FlannBasedMatcher(
            dict(
                algorithm=6,
                table_number=6,
                key_size=12,
                multi_probe_level=1
            ),
            dict(
                checks=100
            )
        )

        h, w = self.pattern.shape

        self.target_points_2d = np.float32(
            [[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)

        h_, w_ = h // 2, w // 2

        self.target_points_3d = np.float32(
            [[-w_+1, -h_+1, 0],
             [-w_+1, h_-1, 0],
             [w_-1, h_-1, 0],
             [w_-1, -h_+1, 0]])

        self.x_ratio = 0.7
        self.y_ratio = 0.7

        self.match_test_ratio = 0.8

        self.knn_parameter = 2

        self.filter = MovingAverage()

        self.tf = tf.TransformListener()

        self.camera_model = image_geometry.PinholeCameraModel()

        self._ap_ctrl = rospy.Subscriber('/apctrl', Empty, self.on_toggle,
                                         queue_size=1)

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

    def on_info(self, data):
        self.info = data

    frame = 0

    def process_image(self, img):
        if self.info is None:
            self.process_data(None)
            return

        image_detect = self.detect(img)

        if len(image_detect.kp) < self.min_points:
            self.process_data(None)
            return img

        matches = self.match(image_detect)
        matched_image_detect = self.filter_train_kp(
            image_detect, matches)
        matched_pattern_detect = self.filter_query_kp(
            self.pattern_detect, matches)

        if len(matches) < self.min_points:
            self.process_data(None)
            return self.draw_match(img, image_detect, matched_image_detect,
                                   success=False)

        transform, mask = cv2.findHomography(
            np.float32(map(attrgetter('pt'), matched_pattern_detect.kp)),
            np.float32(map(attrgetter('pt'), matched_image_detect.kp)),
            cv2.RANSAC, 5.0)

        bbox = cv2.perspectiveTransform(self.target_points_2d, transform)

        if not self.check_match(bbox):
            self.process_data(None)
            return self.draw_match(img, image_detect, matched_image_detect,
                                   bbox, success=False)

        camera_matrix = np.float32(self.info.K).reshape(3, 3)
        camera_distortion = np.float32(self.info.D)

        rot, trans, inl = cv2.solvePnP(
            self.target_points_3d, np.float32(bbox),
            camera_matrix, camera_distortion,
           # iterationsCount=5,
            flags=0#,
           # reprojectionError=20
        )

        self.process_data(trans, img)
        return self.draw_match(img, image_detect, matched_image_detect, bbox,
                               success=True)

    def detect(self, img, *args, **kwargs):
        """Detects keypoints and descriptors of the image

        Returns a `Detect` class instance.

        """
        kp, desc = self.detector.detectAndCompute(img, None, *args, **kwargs)
        return Detect(kp, desc)

    def match(self, image_detect):
        """Matches the detected features against the pattern features"""
        matches = self.matcher.knnMatch(self.pattern_detect.desc,
                                        image_detect.desc,
                                        k=self.knn_parameter)
        good_matches = [i for i in matches if len(i) == 2]
        ratio_matches = [m for m, n in good_matches
                         if m.distance < self.match_test_ratio * n.distance]
        return ratio_matches

    def draw_match(self, img, image_detect, good_image_detect,
                   bbox=None, success=True):
        """Draws the found and matched keypoints and the target rectangle"""
        if success:
            color = (0, 255, 0)
        else:
            color = (0, 0, 255)
        from copy import copy
        img2 = copy(img)
        img = cv2.drawKeypoints(img, image_detect.kp, img2,
                                color=(0, 20, 20), flags=0)
        img = cv2.drawKeypoints(img, good_image_detect.kp, img2,
                                color=color, flags=0)

        if bbox is not None:
            cv2.polylines(img, [np.int32(bbox)], True, color, 3)

        return img

    def check_match(self, points):
        """Returns True if the given vertexes for a valid rectandle"""
        if len(points) != 4:
            return False

        a = self.distance(points[0][0], points[1][0])
        b = self.distance(points[1][0], points[2][0])
        c = self.distance(points[2][0], points[3][0])
        d = self.distance(points[3][0], points[0][0])

        if (a / c < self.x_ratio or c / a < self.x_ratio or
                b / d < self.y_ratio or d / b < self.y_ratio):
            return False

        return True

    @staticmethod
    def filter_query_kp(query_detect, matches):
        """Filters the matched `query_detect` keypoints

        Returns a `Detect` class instance with filtered keypoints iterable
        and no descriptors.

        """
        return Detect([query_detect.kp[m.queryIdx] for m in matches])

    @staticmethod
    def filter_train_kp(train_detect, matches):
        """Filters the matched `train_detect` keypoints

        Returns a `Detect` class instance with filtered keypoints iterable
        and no descriptors.

        """
        return Detect([train_detect.kp[m.trainIdx] for m in matches])

    @staticmethod
    def distance(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

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
