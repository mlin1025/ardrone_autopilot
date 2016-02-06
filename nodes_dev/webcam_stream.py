#!/usr/bin/env python
#
# Webcamera streaming node for debug purpose
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#
"""
Node for streaming information from USB webcamera.

It's common use for CV system debugging.


Outputs
-------

* webcam/image -- webcam video stream.


Parameters
----------

* ~camera_index = -1 [int] -- camera index for OpenCV
  (`-1` = any available camera).
* ~stream_rate = 15 [uint] -- stream frames per second rate.
* ~swap_red_blue = False [bool] -- set this to `True` if you need to swap
  red and blue channels (required for some cameras).
* ~encoding = "bgr8" [str] -- video encoding used by bridge.


"""

import cv
import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image


class WebCamNode(object):
    def __init__(self,
                 camera_index=-1,
                 rate=15,
                 swap_red_blue=False,
                 encoding='bgr8'):
        self.swap_red_blue = swap_red_blue
        self.encoding = encoding
        self.camera_index = camera_index
        self.capture = cv2.VideoCapture(camera_index)
        self.rate = rospy.Rate(rate)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('webcam/image', Image, queue_size=10)

    def frame(self):
        """Acquire and publish one more frame"""
        ret_val, frame = self.capture.read()
        if self.swap_red_blue:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        image_message = self.bridge.cv2_to_imgmsg(frame, self.encoding)
        self.pub.publish(image_message)
        cv.WaitKey(1)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('webcam_node', anonymous=True)

    camera_index = rospy.get_param('~camera_index', -1)
    stream_rate = rospy.get_param('~stream_rate', 15)
    swap_red_blue = rospy.get_param('~swap_red_blue', False)
    encoding = rospy.get_param('~encoding', 'bgr8')

    rospy.loginfo(
        'Starting webcam (cam=%s rate=%s)' % (camera_index, stream_rate))

    node = WebCamNode(camera_index, stream_rate, swap_red_blue, encoding)
    try:
        while not rospy.is_shutdown():
            node.frame()
    except rospy.ROSInterruptException:
        pass
