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
from threading import Lock

from PySide import QtCore, QtGui

import rospy

from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist


# Message groups
grid = [
    'drone.state',
    None,
    'drone.battery',
    None,
    ['od.pos.x',
     'od.pos.y',
     'od.pos.z'],
    ['od.ori.x',
     'od.ori.y',
     'od.ori.z',
     'od.ori.w'],
    None,
    ['imu.ori.x',
     'imu.ori.y',
     'imu.ori.z'],
    ['imu.vel.x',
     'imu.vel.y',
     'imu.vel.z'],
    ['imu.acc.x',
     'imu.acc.y',
     'imu.acc.z']
]


class DroneController(object):
    def __init__(self, command_rate=20):
        self.status = -1

        rospy.Subscriber('/ardrone/navdata', Navdata, self.on_navdata)

        self.land_topic = rospy.Publisher('/ardrone/land', Empty, queue_size=5)
        self.takeoff_topic = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
        self.reset_topic = rospy.Publisher('/ardrone/reset', Empty, queue_size=5)

        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.command = Twist()

        rospy.on_shutdown(self.land)

    def on_navdata(self, navdata):
        self.status = navdata.state

    def takeoff(self):
        if(self.status == 2):
            self.takeoff_topic.publish(Empty())

    def land(self):
        self.land_topic.publish(Empty())

    def reset(self):
        self.reset_topic.publish(Empty())

    def hover(self):
        self.command.linear.x = 0
        self.command.linear.y = 0
        self.command.linear.z = 0
        self.command.angular.x = 0
        self.command.angular.y = 0
        self.command.angular.z = 0

    def set_commant(self, roll=None, pitch=None, yaw=None, z_vel=None):
        if pitch is not None:
            self.command.linear.x = pitch
        if roll is not None:
            self.command.linear.y = roll
        if z_vel is not None:
            self.command.linear.z = z_vel
        if yaw is not None:
            self.command.angular.z = yaw

    def send_command(self):
        if self.status in [3, 4, 7]:
            self.cmd.publish(self.command)


class Messages(object):
    def __init__(self, message_display_time, *args):
        self.message_structure = args
        self.messages_named = {}
        self.messages_queue = deque()
        self.message_display_time = message_display_time
        self.lock = Lock()

    def messages_put(self, messages):
        """Add new messages to UI"""
        with self.lock:
            for message, name in messages:
                if name is None:
                    self.messages_queue.append((message, rospy.get_time()))
                else:
                    self.messages_named[name] = message

    def message_put(self, message, name=None):
        """Add one new message to UI"""
        self.messages_put([(message, name), ])

    def messages_flush(self):
        with self.lock:
            messages = {}

    def render(self, image):
        """Prints all messages onto given image"""
        self.clean_queue()

        painter = QtGui.QPainter()
        painter.begin(image)
        painter.setPen(QtGui.QColor(255, 255, 255))
        painter.setBrush(QtGui.QColor(0, 255, 0))

        w, h = image.width(), image.height()
        x, y = int(w * .05), int(h * .05)
        mw = w / 5

        with self.lock:
            for entry in self.message_structure:
                if entry is None:
                    entry = []
                if not isinstance(entry, (list, tuple)):
                    entry = [entry]
                for name in entry:
                    text = '%s: %s' % (name,
                                       self.messages_named.get(name, '-'))
                    painter.drawText(x, y, text)
                    y += 15
                y += 5
                if y > int(h * .9):
                    y = int(h * .05)
                    x += mw

            x, y = int(w - mw), int(h * .05)
            for text, time in self.messages_queue:
                painter.drawText(x, y, text)
                y += 15
                if y > int(h * .9):
                    y = int(h * .05)
                    x -= mw

        painter.end()

    def clean_queue(self):
        """Removes all outdated messages from the queue"""
        with self.lock:
            while (self.messages_queue and
                   rospy.get_time() - self.messages_queue[0][1] >
                   self.message_display_time / 1000):
                self.messages_queue.popleft()


class UInode(QtGui.QMainWindow):
    status = {
        0: 'Emergency',
        1: 'Initialized',
        2: 'Landed',
        3: 'Flying',
        4: 'Hovering',
        5: 'Test (?)',
        6: 'Taking Off',
        7: 'Going to Hover Mode',
        8: 'Landing',
        9: 'Looping (?)',
    }

    def __init__(self,
                 controller=None,
                 message_display_time=5000,
                 coneection_check_period=500,
                 grid=grid,
                 fps=50):
        super(UInode, self).__init__()

        self.controller = controller

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.z_vel = 0

        self.messages = Messages(message_display_time, *grid)
        self.messages_named_template = re.compile(
            r'((?P<name>[a-zA-Z0-9_-]+)::)?(?P<message>.*)')

        self.setWindowTitle('ARdrone camera')
        self.image_box = QtGui.QLabel(self)
        self.setCentralWidget(self.image_box)

        self.image = None
        self.image_lock = Lock()

        self.communication_since_timer = False
        self.connected = False
        self.messages.message_put('Offline', 'drone.state')

        self.connection_check_timer = QtCore.QTimer(self)
        self.connection_check_timer.timeout.connect(self.on_connection_check)
        self.connection_check_timer.start(coneection_check_period)

        self.redraw_timer = QtCore.QTimer(self)
        self.redraw_timer.timeout.connect(self.on_redraw)
        self.redraw_timer.start(1000 / fps)

        rospy.Subscriber('/ardrone/navdata', Navdata, self.on_navdata)
        rospy.Subscriber('/ui/message', String, self.on_ui_request)
        rospy.Subscriber('camera/image', Image, self.on_video_update)
        rospy.Subscriber('odom', Odometry, self.on_odometry)
        rospy.Subscriber('imu', Imu, self.on_imu)

    def on_ui_request(self, message):
        """We have spetial `ui/message` topic where any node can send
        any message and that message will be displayed.

        By default messages are stacked in queue and displayed for a while.

        Messages which match mask `([a-zA-Z0-9_-])::(.*)` will be displayed
        permanently. Newer messages will overwrite older messages
        with the same name.

        """
        match = self.messages_named_template.match(message.data)
        self.messages.message_put(**match.groupdict())

    def on_odometry(self, data):
        """Update odometry data"""
        self.communication_since_timer = True

        self.messages.messages_put([
            (data.pose.pose.position.x, 'od.pos.x'),
            (data.pose.pose.position.y, 'od.pos.y'),
            (data.pose.pose.position.z, 'od.pos.z'),

            (data.pose.pose.orientation.x, 'od.ori.x'),
            (data.pose.pose.orientation.y, 'od.ori.y'),
            (data.pose.pose.orientation.z, 'od.ori.z'),
            (data.pose.pose.orientation.w, 'od.ori.w'),
        ])

    def on_imu(self, data):
        """Update IMU data"""
        self.communication_since_timer = True

        self.messages.messages_put([
            (data.orientation.x, 'imu.ori.x'),
            (data.orientation.y, 'imu.ori.y'),
            (data.orientation.z, 'imu.ori.z'),

            (data.angular_velocity.x, 'imu.vel.x'),
            (data.angular_velocity.y, 'imu.vel.y'),
            (data.angular_velocity.z, 'imu.vel.z'),

            (data.linear_acceleration.x, 'imu.acc.x'),
            (data.linear_acceleration.y, 'imu.acc.y'),
            (data.linear_acceleration.z, 'imu.acc.z'),
        ])

    def on_navdata(self, data):
        self.communication_since_timer = True

        self.messages.messages_put([
            (self.status.get(data.state, 'unknown'), 'drone.state'),
            (data.batteryPercent, 'drone.battery'),
        ])

    def on_video_update(self, data):
        """On each frame we save new picture for future rendering"""
        self.communication_since_timer = True

        image = QtGui.QImage(data.data,
                             data.width,
                             data.height,
                             QtGui.QImage.Format_RGB888)
        # Try removing the next line if you have problems with colors
        # image = QtGui.QImage.rgbSwapped(image)

        with self.image_lock:
            self.image = image

    def on_connection_check(self):
        """An obvious way to check if the drone is online"""
        if self.communication_since_timer and not self.connected:
            self.messages.message_put('Online', 'drone.state')
        elif not self.communication_since_timer:
            self.image = None
            self.messages.messages_flush()
            self.messages.message_put('Offline', 'drone.state')
        self.connected = self.communication_since_timer
        self.communication_since_timer = False

    def on_redraw(self):
        image = None
        with self.image_lock:
            if self.image is not None:
                image = QtGui.QPixmap.fromImage(self.image)
            else:
                image = QtGui.QPixmap(640, 360)
                image.fill(QtGui.QColor(50, 50, 50))

        self.messages.render(image)

        self.resize(image.width(), image.height())
        self.image_box.setPixmap(image)

    def keyPressEvent(self, event):
        key = event.key()

        if event.isAutoRepeat() or self.controller is None:
            return

        if key == QtCore.Qt.Key.Key_R:
            self.controller.reset()
        elif key == QtCore.Qt.Key.Key_L:
            self.controller.takeoff()
        elif key == QtCore.Qt.Key.Key_T:
            self.controller.land()
        elif key == QtCore.Qt.Key.Key_H:
            self.controller.hover()
        else:
            if key == QtCore.Qt.Key.Key_A:
                self.yaw = 1
            elif key == QtCore.Qt.Key.Key_D:
                self.yaw = -1

            elif key == QtCore.Qt.Key.Key_W:
                self.pitch = 1
            elif key == QtCore.Qt.Key.Key_S:
                self.pitch = -1

            elif key == QtCore.Qt.Key.Key_Q:
                self.roll = 1
            elif key == QtCore.Qt.Key.Key_E:
                self.roll = -1

            elif key == QtCore.Qt.Key.Key_BracketRight:
                self.z_vel = 1
            elif key == QtCore.Qt.Key.Key_BracketLeft:
                self.z_vel = -1

        self.controller.set_commant(self.roll, self.pitch, self.yaw, self.z_vel)
        self.controller.send_command()

    def keyReleaseEvent(self, event):
        key = event.key()

        if event.isAutoRepeat() or self.controller is None:
            return

        if key == QtCore.Qt.Key.Key_A:
            self.yaw = 0
        elif key == QtCore.Qt.Key.Key_D:
            self.yaw = 0

        elif key == QtCore.Qt.Key.Key_W:
            self.pitch = 0
        elif key == QtCore.Qt.Key.Key_S:
            self.pitch = 0

        elif key == QtCore.Qt.Key.Key_Q:
            self.roll = 0
        elif key == QtCore.Qt.Key.Key_E:
            self.roll = 0

        elif key == QtCore.Qt.Key.Key_BracketRight:
            self.z_vel = 0
        elif key == QtCore.Qt.Key.Key_BracketLeft:
            self.z_vel = 0

        self.controller.set_commant(self.roll, self.pitch, self.yaw, self.z_vel)
        self.controller.send_command()


if __name__ == '__main__':
    import sys

    rospy.init_node('ui_node')

    app = QtGui.QApplication(sys.argv)

    ui = UInode(DroneController())
    ui.show()

    status = app.exec_()
    sys.exit(status)
