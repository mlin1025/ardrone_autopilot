#!/usr/bin/env python
#
# UI node for visualizing ARdrone state
#
# This code is a part of `ardrone_autopilot` project
# which is distributed under the MIT license.
# See `LICENSE` file for details.
#

import re
from collections import deque, OrderedDict

import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry


class UInode(object):
    def __init__(self, receive_user_input=True, time_queue_display=5):
        self.receive_user_input = receive_user_input

        self.time_queue_display = time_queue_display
        self.messages_queue = deque()
        self.messages_named = OrderedDict()
        self.named_message_template = re.compile(
            r'((?P<name>[a-zA-Z0-9_-]+)::)?(?P<message>.*)')

        self.bridge = CvBridge()

        self.takeoff = rospy.Publisher('ardrone/takeoff', Empty, queue_size=10)
        self.land = rospy.Publisher('ardrone/land', Empty, queue_size=10)
        self.reset = rospy.Publisher('ardrone/reset', Empty, queue_size=10)

        rospy.Subscriber("arp/ui/debug_info", String, self.on_ui_request)
        # rospy.Subscriber("ardrone/odometry", Odometry, self.on_odometry)
        rospy.Subscriber("ardrone/imu", Imu, self.on_imu)
        rospy.Subscriber("ardrone/front/image_raw", Image, self.on_video_update)

    def message_put(self, message, name=None):
        """Add a new message to UI"""
        if name is None:
            self.messages_queue.append((message, rospy.get_time()))
        else:
            self.messages_named[name] = message

    def on_ui_request(self, message):
        """We have spetial `arp/ui/debug_info` topic where any node can send
        any message and that message will be displayed.

        By default messages are stacked in queue and displayed for a while.

        Messages which match mask `([a-zA-Z0-9_-])::(.*)` will be displayed
        permanently. Newer messages will overwrite older messages
        with the same name.

        Be advised that messages are only shown after videoframe update.

        """
        match = self.named_message_template.match(message.data)
        self.message_put(**match.groupdict())

    def on_odometry(self, data):
        """Update odometry data"""
        self.message_put(data.header.seq, 'odometry-seq')

        self.message_put(data.pose.pose.position.x, 'od.pos.x')
        self.message_put(data.pose.pose.position.y, 'od.pos.y')
        self.message_put(data.pose.pose.position.z, 'od.pos.z')

        self.message_put(data.pose.pose.orientation.x, 'od.ori.x')
        self.message_put(data.pose.pose.orientation.y, 'od.ori.y')
        self.message_put(data.pose.pose.orientation.z, 'od.ori.z')
        self.message_put(data.pose.pose.orientation.w, 'od.ori.w')

    def on_imu(self, data):
        """Update IMU data"""
        self.message_put(data.header.seq, 'imu-seq')

        self.message_put(data.orientation.x, 'imu.ori.x')
        self.message_put(data.orientation.y, 'imu.ori.y')
        self.message_put(data.orientation.z, 'imu.ori.z')

        self.message_put(data.angular_velocity.x, 'imu.vel.x')
        self.message_put(data.angular_velocity.y, 'imu.vel.y')
        self.message_put(data.angular_velocity.z, 'imu.vel.z')

        self.message_put(data.linear_acceleration.x, 'imu.acc.x')
        self.message_put(data.linear_acceleration.y, 'imu.acc.y')
        self.message_put(data.linear_acceleration.z, 'imu.acc.z')

    def on_video_update(self, data):
        """On each frame render new picture and add messages there"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
            return

        self.print_messages(cv_image)

        cv2.imshow("ARdrone camera", cv_image)
        key = cv2.waitKey(10)
        if key != -1:
            self.process_user_input(key)

    def print_messages(self, cv_image):
        """Prints all messages onto given image"""
        w, h, _ = cv_image.shape
        x, y = int(w * .1), int(h * .1)
        mw = 100
        for name, message in self.messages_named.items():
            text = '%s: %s' % (name, message)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, text, (x, y), font, .5, 0, 2)
            cv2.putText(cv_image, text, (x, y), font, .5, (255, 255, 255), 1)
            y += 15
            if y > int(h * .9):
                y = int(h * .1)
                x += mw

        x, y = int(w), int(h * .1)
        for text, time in self.messages_queue:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, text, (x, y), font, .5, 0, 2)
            cv2.putText(cv_image, text, (x, y), font, .5, (255, 255, 255), 1)
            y += 15
            if y > int(h * .9):
                y = int(h * .1)
                x -= mw

        self.clean_queue()

    def clean_queue(self):
            while (self.messages_queue and
                   rospy.get_time() - self.messages_queue[0][1] >
                   self.time_queue_display):
                print(rospy.get_time() - self.messages_queue[0][1])
                self.messages_queue.popleft()

    def process_user_input(self, key):
        if key == ord('l'):
            print('LAUNCH')
            self.takeoff.publish(Empty())
        elif key == ord('r'):
            print('RESET')
            self.reset.publish(Empty())
        elif key == ord('d'):
            print('DROP')
            self.land.publish(Empty())


if __name__ == '__main__':
    ## Do we really need all this?
    # import argparse
    # parser = argparse.ArgumentParser(add_help=False)
    # parser.add_argument(
    #     '-a', '--anonymous', action='store_true',
    #     help='Run this node with uniq name so that you can '
    #          'run multiple instances of UI. Anonymous nodes '
    #          'cannot receive and process user input.'
    # )
    # args = parser.parse_args()
    # rospy.init_node('listener', anonymous=args.anonymous)

    rospy.init_node('listener')
    ui = UInode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
