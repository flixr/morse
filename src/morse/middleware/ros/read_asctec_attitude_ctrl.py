import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('uav_msgs'); roslib.load_manifest('rosgraph_msgs')
import rospy
import std_msgs
import math
import mathutils
from uav_msgs.msg import AttitudeCtrlInput

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, AttitudeCtrlInput, callback_attitude, component_instance))

def callback_attitude(data, component_instance):
    """ this function is called as soon as AttitudeCtrlInput messages are published on the specific topic """
    component_instance.local_data["pitch"] = data.pitch
    component_instance.local_data["roll"] = data.roll
    component_instance.local_data["yaw"] = data.yaw
    component_instance.local_data["thrust"] = data.thrust
    component_instance.local_data["ctrl"] = data.ctrl

    logger.debug("new RPY thrust setpoint: (%.2f %.2f %.3f %3f)" % (math.degrees(data.roll), math.degrees(data.pitch), data.yaw / 20.47, data.thrust))

def read_attitude(self, component_instance):
    """ dummy function AsctecAttitudeCtrl controller """
