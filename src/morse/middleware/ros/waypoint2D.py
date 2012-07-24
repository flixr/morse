import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import std_msgs
from geometry_msgs.msg import Pose2D

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(self.topic_name(component_instance), Pose2D, callback_wp, component_instance))

def callback_wp(data, component_instance):
    """ this function is called as soon as Pose2Ds are published on the specific topic """

    logger.debug("Received Pose2D: < %s, %s, %s > on topic %s"% \
            (data.x, data.y, data.theta, self.topic_name(component_instance))
    component_instance.local_data["x"] = data.x
    component_instance.local_data["y"] = data.y
    component_instance.local_data["z"] = data.theta

def read_waypoint(self, component_instance):
    """ dummy function for Waypoints """
