import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')
import rospy
import std_msgs
from sensor_msgs.msg import JointState
import bge
import math
import mathutils

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
     # Add the new method to the component
    component_instance.output_functions.append(function)
 
    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(self.topic_name(component_instance), JointState))
        
    logger.info('######## ROS JOINTSTATE PUBLISHER INITIALIZED ########')

def post_jointState(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = 'head_pan_joint', 'head_tilt_joint'

    js.position = [component_instance.local_data['pan'], component_instance.local_data['tilt']] 
               
    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == self.topic_name(component_instance):
            topic.publish(js)
