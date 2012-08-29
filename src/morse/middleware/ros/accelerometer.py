import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
from geometry_msgs.msg import Twist
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
    if mw_data[1] == "post_twist":  
        self._topics.append(rospy.Publisher(self.topic_name(component_instance), Twist))

    logger.info('######## ACCELEROMETER-SENSOR INITIALIZED ########')

def post_twist(self, component_instance):
    """ Publish the data of the acceleromter sensor as a ROS Twist message
       
    Only the velocity part is exported.
    """
    twist = Twist()
    
    # Fill twist-msg with the values from the sensor
    twist.linear.x = component_instance.local_data['velocity'][0]
    twist.linear.y = component_instance.local_data['velocity'][1]
    twist.linear.z = component_instance.local_data['velocity'][2]
    
    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == self.topic_name(component_instance): 
            topic.publish(twist)
