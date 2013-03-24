import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('uav_msgs'); roslib.load_manifest('visual_odometry_msgs')

from uav_msgs.msg import GpioEvent
from visual_odometry_msgs.msg import VisualOdometry
from morse.middleware.ros import ROSPublisher, mathutils
import rospy
import math

class VisualOdometryPublisher(ROSPublisher):
    """ Publish the data of the DeltaPose sensor as VisualOdometry message."""
    ros_class = VisualOdometry

    def initialize(self):
        ROSPublisher.initialize(self)
        self.prev_time = rospy.Time()
        self.pos_err = 0.003
        self.rot_err = math.radians(0.3)

    def default(self, ci='unused'):

        now = rospy.Time.now()

        if self.prev_time != rospy.Time():
            odom = VisualOdometry()
            odom.begin.data = self.prev_time
            odom.end.data = now
            odom.pose.position.x = self.data['x']
            odom.pose.position.y = self.data['y']
            odom.pose.position.z = self.data['z']
            odom.pose.orientation = self.data['orientation']
            odom.error[0] = self.pos_err
            odom.error[1] = self.pos_err
            odom.error[2] = self.pos_err
            odom.error[3] = self.rot_err
            odom.error[4] = self.rot_err
            odom.error[5] = self.rot_err

            self.publish(odom)

        else:
            logger.info('Skipping first one...')

        self.prev_time = now


class GpioEventPublisher(ROSPublisher):
    """ Publish a GpioEvent message."""
    ros_class = GpioEvent

    def default(self, ci='unused'):
        trigger = GpioEvent()
        trigger.stamp = rospy.Time.now()
        trigger.gpio = 146  # the gpio used on the pelican for the cam trigger
        trigger.edgetype = 2  # rising edge (0x01) , falling edge (0x02) or both (0x03)

        self.publish(trigger)
