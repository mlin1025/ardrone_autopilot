#!/usr/bin/env python
#
# UI node for visualizing ARdrone state
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#

#import cv
import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image


class WebCamNode(object):
    def __init__(self, camera_index=-1, rate=15):
        self.camera_index = camera_index
        
        self.capture = cv2.VideoCapture(camera_index)
        
        self.rate = rospy.Rate(rate)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/image', Image, queue_size=1)

    def frame(self):
        """Acquire and publish one more frame"""
        ret_val, frame = self.capture.read()
        image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub.publish(image_message)
        cv2.imshow('t', frame)
        cv2.cv.WaitKey(1)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('webcam_stream')
    node = WebCamNode()
    try:
        while not rospy.is_shutdown():
            node.frame()
    except rospy.ROSInterruptException:
        pass
