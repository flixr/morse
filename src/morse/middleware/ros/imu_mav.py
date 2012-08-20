import logging; logger = logging.getLogger("morse." + __name__)

import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('uav_msgs');

import rospy
import std_msgs
from uav_msgs.msg import ImuMav

import bge
import mathutils
import math

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    if mw_data[1] == "post_imu_mav":
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, ImuMav))

    self._seq = 0

    logger.info('######## Initialized ROS ImuMav sensor ########')


def post_imu_mav(self, component_instance):
    """ Publish the data of the IMU sensor as a custom ROS ImuMav message
    """
    parent = component_instance.robot_parent
    parent_name = parent.blender_obj.name
    current_time = rospy.Time.now()

    imu = ImuMav()
    imu.header.seq = self._seq
    imu.header.stamp = current_time
    imu.header.frame_id = "/imu"

    imu.angular_velocity.x = component_instance.local_data['angular_velocity'][0]
    imu.angular_velocity.y = component_instance.local_data['angular_velocity'][1]
    imu.angular_velocity.z = component_instance.local_data['angular_velocity'][2]

    imu.linear_acceleration.x = component_instance.local_data['linear_acceleration'][0]
    imu.linear_acceleration.y = component_instance.local_data['linear_acceleration'][1]
    imu.linear_acceleration.z = component_instance.local_data['linear_acceleration'][2]

    imu.magnetometer.x = 0.0
    imu.magnetometer.y = 0.0
    imu.magnetometer.z = 0.0

    imu.baro_height = 0.0


    for topic in self._topics:
        # publish the message on the correct w
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(imu)

    self._seq += 1