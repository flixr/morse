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
    # Add the new method to the component
    component_instance.output_functions.append(function)
    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name

    logger.setLevel(logging.DEBUG)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(self.topic_name(component_instance), Height))
    self._seq = 0

    # Extract the Middleware parameters
    # additional parameter should be a dict
    # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
    frame_id = '/map'
    try:
        frame_id = mw_data[3].get("frame_id", frame_id)
    except:
        pass

    # create a new dictionary for this sensor if necessary
    if component_name not in self._properties:
        self._properties[component_name] = {}
    # store the frame id in the dict
    self._properties[component_name]['frame_id'] = frame_id

    logger.info('######## ROS HEIGHT PUBLISHER INITIALIZED ########')

def post_height(self, component_instance):
    """ Publish the data of the altitude sensor as a ROS Height message.

    """
    component_name = component_instance.blender_obj.name

    height = Height()
    height.header.stamp = rospy.Time.now()
    height.header.seq = self._seq
    height.header.frame_id = self._properties[component_name]['frame_id']
    height.height = component_instance.local_data['height']
    height.distance = component_instance.local_data['height']
    height.height_variance = 0.05

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == self.topic_name(component_instance):
            topic.publish(height)

    self._seq = self._seq + 1
