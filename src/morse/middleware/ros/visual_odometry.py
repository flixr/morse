import logging; logger = logging.getLogger("morse." + __name__)

import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('uav_msgs');

import rospy
from std_msgs.msg import Time
from uav_msgs.msg import VisualOdometry, GpioEvent

import mathutils
import math

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Add the new method to the component
    component_instance.output_functions.append(function)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    if mw_data[1] == "post_visual_odometry":
        self._topics.append(rospy.Publisher(self.topic_name(component_instance), VisualOdometry))

    # add GpioTrigger topic
    self._topics.append(rospy.Publisher("cam_trigger", GpioEvent))

    self.prev_time = rospy.Time()

    self.pos_err = 0.01
    self.rot_err = math.radians(1)

    logger.info('######## Initialized ROS VisualOdometry sensor ########')


def post_visual_odometry(self, component_instance):
    """ Publish the data of the DeltaPose sensor as a custom ROS VisualOdometry message
    """

    now = rospy.Time.now()

    trigger = GpioEvent()
    trigger.stamp = now
    trigger.gpio = 146 # the gpio used on the pelican for the cam trigger
    trigger.edgetype = 2 # rising edge (0x01) , falling edge (0x02) or both (0x03)

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/cam_trigger"):
            topic.publish(trigger)

    if self.prev_time != rospy.Time():
        odom = VisualOdometry()
        odom.begin.data = self.prev_time
        odom.end.data = now
        odom.pose.position.x = component_instance.local_data['x']
        odom.pose.position.y = component_instance.local_data['y']
        odom.pose.position.z = component_instance.local_data['z']
        odom.pose.orientation = component_instance.local_data['orientation']
        odom.error[0] = self.pos_err
        odom.error[1] = self.pos_err
        odom.error[2] = self.pos_err
        odom.error[3] = self.rot_err
        odom.error[4] = self.rot_err
        odom.error[5] = self.rot_err

        for topic in self._topics:
            # publish the message on the correct w
            if str(topic.name) == self.topic_name(component_instance):
                topic.publish(odom)

    else:
        logger.debug('Skipping first one...')

    self.prev_time = now
