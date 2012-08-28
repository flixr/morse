import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('mav_msgs');
import rospy
from mav_msgs.msg import Height

import math
import mathutils

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)
    
    logger.setLevel(logging.DEBUG)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Height))
    self._seq = 0

    (loc, rot, scale) = component_instance.robot_parent.position_3d.transformation3d_with(component_instance.position_3d).matrix.decompose()
    
    # store body to imu rotation and translation
    self.rot_b2i = rot
    self.trans_b2i = loc

    logger.info('######## ROS HEIGHT PUBLISHER INITIALIZED ########')

def post_height(self, component_instance):
    """ Publish the data of the altitude sensor as a ROS Height message.

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name

    height = Height()
    height.header.stamp = rospy.Time.now()
    height.header.seq = self._seq
    # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
    height.header.frame_id = ('/' + parent_name)
    height.height = component_instance.local_data['height']
    height.distance = component_instance.local_data['height']
    height.height_variance = 0.01

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_name):
            topic.publish(height)
    
    self._seq = self._seq + 1
