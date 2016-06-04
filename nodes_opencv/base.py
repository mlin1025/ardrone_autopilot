#!/usr/bin/env python
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""Base class for testing computer vision systems

This is the base class which providing a fairly simple functionality:
it receives an image stream and call a callback function per each frame.


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
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo


class BaseStreamHandler(object):
    def __init__(self, encoding='bgr8', show=False):
        self.pub = rospy.Publisher('out/image', Image, queue_size=1)
        rospy.Subscriber('in/image', Image, self.__on_image, queue_size=1)
        rospy.Subscriber('in/info', CameraInfo, self.__on_info, queue_size=1)
        self.bridge = CvBridge()
        self.encoding = encoding
        self.show = show
        self.info = None

    def __on_info(self, data):
        self.info = data

    def __on_image(self, data):
        """
        Hidden callback process image so that user can work with opencv
        without taking care about conversion etc.

        """
        img = self.bridge.imgmsg_to_cv2(data, self.encoding)
        img_out = self.on_image(img)
        if img_out is not None:
            if self.show:
                cv2.imshow('Result', img_out)
            else:
                img_msg = self.bridge.cv2_to_imgmsg(img_out, self.encoding)
                self.pub.publish(img_msg)

        cv2.cv.WaitKey(1)

    def on_image(self, img):
        """User code lives here"""
        raise NotImplementedError('this method should be ovarloaded')

    @classmethod
    def launch_node(cls, **kwargs):
        """A common code for launching node (blocks!)"""

        rospy.init_node('opencv_test_node', anonymous=True)

        launch_args = {
            'encoding': rospy.get_param('~encoding', 'bgr8'),
            'show': rospy.get_param('~show', False),
        }

        for k, v in kwargs.items():
            if isinstance(v, (list, tuple)):
                v = rospy.get_param(*v)
            else:
                v = rospy.get_param(v)
            launch_args[k] = v

        node = cls(**launch_args)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
