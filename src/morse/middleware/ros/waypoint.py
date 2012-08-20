import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs');
import rospy
import std_msgs
from geometry_msgs.msg import Pose
import mathutils

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Pose, callback_wp, component_instance))

def callback_wp(data, component_instance):
    """ this function is called as soon as a Pose are published on the specific topic """

    component_instance.local_data["x"] = data.position.x
    component_instance.local_data["y"] = data.position.y
    component_instance.local_data["z"] = data.position.z
    quaternion = mathutils.Euler((data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z))
    euler = quaternion.to_euler()
    component_instance.local_data["psi"] = euler.z

def read_waypoint(self, component_instance):
    """ dummy function for Waypoints """
