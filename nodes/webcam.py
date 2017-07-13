#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

import cv2
import cookielib
import urllib
import math
import numpy as np
import os
import time
import urllib2

from threading import Thread

from cv_bridge import CvBridge, CvBridgeError



class WebcamNode(object):
    def __init__(self, url='http://192.168.0.12:8080/videofeed'):
        self.cam_started = False
        self.cam_threads = 0
        self.url = url
        self.pub = rospy.Publisher('/in/image', Image, queue_size=10)
        rospy.Subscriber('/webcam/shutdown', Empty, self.shutdown)
        rospy.init_node('webcam')
        self.bridge = CvBridge()
        self.encoding = rospy.get_param('~encoding', '8UC3')

    def start_webcam(self):
    #   def start_webcam_thread():
        url = self.url
        if self.cam_threads >= 1:
            print 'Already started!'
            return 0
        try:
            stream = urllib.urlopen(url)
            bytes = ''
            self.cam_started = True
            self.cam_threads += 1
            print('CAM: STARTED')
        except IOError:
            print('CAM: IOError')
            self.cam_threads -= 1
            self.cam_started = False
            try:
                stream.close()
            except:
                pass
            return 0
    
        while self.cam_started:
            try:
                # print ('.')
                bytes += stream.read(1024)
                if bytes == '':
                    print('CAM: URL Error. Stopping camera')
                    stream.close()
                    self.cam_threads -= 1
                    self.cam_started = False
                    return 0
                a = bytes.find('\xff\xd8')
                b = bytes.find('\xff\xd9')
                if a != -1 and b != -1:
                    jpg = bytes[a:b+2]
                    bytes = bytes[b+2:]
                    try:
                        i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), 1)
                        self.report_img = i
                        out_image = self.bridge.cv2_to_imgmsg(i, self.encoding)
                        self.pub.publish(out_image)
                    except Exception as e:
                        print(e)
                        print('CAM: Image error')
                    if cv2.waitKey(2) == 27:
                        stream.close()
                        self.cam_threads -= 1
                        self.cam_started = False
                        return 0
            except Exception as e:
                print(e)
                if (isinstance(e, KeyboardInterrupt)):
                    self.cam_started = False
    		    print('CAM: kill')
                print('CAM: Unknown error. Exiting stream...')
                stream.close()
                time.sleep(0.1)
                return 0
            
         #   Thread(target=start_webcam_thread).run()
            

    ### Get single frame
    def get_img(self):
        return self.report_img

    def shutdown(self, x):
        self.cam_started = False


if __name__ == "__main__":
    rospy.loginfo('Starting webcam')
    node = WebcamNode(url='http://192.168.43.1:8080/videofeed')
    node.start_webcam()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
