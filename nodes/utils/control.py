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
import config

class Autopilot(object):
    def __init__(self):
        self.controller = DroneController()

    def start(self):
        while
        self.controller.send_vel(x = config.x, y = config.y)

def callback(data):
    print(data.data)
   # rospy.loginfo(rospy.get_caller_id() + "%s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("point", String, callback)
    print("took")
    rospy.spin()

if __name__ == '__main__':
    import sys
    listener()
