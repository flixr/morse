import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs'); roslib.load_manifest('uav_msgs');
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from uav_msgs.msg import GpioEvent

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

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name + "/image", Image))
    self._seq = 0

    # Add camera_info topic
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name + "/camera_info", CameraInfo))
    
    # add GpioTrigger topic
    self._topics.append(rospy.Publisher("cam_trigger",GpioEvent))
    
    (loc, rot, scale) = component_instance.robot_parent.position_3d.transformation3d_with(component_instance.position_3d).matrix.decompose()
    
    # in body ned frame
    logger.debug("camera trans (%.4f, %.4f, %.4f)" % (loc.x, -loc.y, -loc.z))
    quat = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(180.0)) * rot
    logger.debug("camera quat (%.4f, %.4f, %.4f, %.4f)" % tuple(quat))
    logger.debug("camera euler (%.4f, %.4f, %.4f)" % tuple(math.degrees(a) for a in quat.to_euler()))

    logger.info('######## ROS IMAGE PUBLISHER INITIALIZED ########')

def post_image_and_trigger(self, component_instance):
    """ Publish the data of the Camera as a ROS-Image message.

    """
    image_local = component_instance.local_data['image']
    if not image_local or image_local == '' or not image_local.image or not component_instance.capturing:
        return # press [Space] key to enable capturing

    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name
    
    trigger = GpioEvent()
    trigger.stamp = rospy.Time.now()
    trigger.gpio = 146 # the gpio used on the pelican for the cam trigger
    trigger.edgetype = 2 # rising edge (0x01) , falling edge (0x02) or both (0x03)
    
    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/cam_trigger"):
            topic.publish(trigger)

    image = Image()
    image.header.stamp = rospy.Time.now()
    image.header.seq = self._seq
    # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
    image.header.frame_id = ('/' + parent_name + '/base_image')
    image.height = component_instance.image_height
    image.width = component_instance.image_width
    image.encoding = 'rgba8'
    image.step = image.width * 4
    # NOTE: Blender returns the image as a binary string encoded as RGBA
    # sensor_msgs.msg.Image.image need to be len() friendly
    # TODO patch ros-py3/common_msgs/sensor_msgs/src/sensor_msgs/msg/_Image.py
    # to be C-PyBuffer "aware" ? http://docs.python.org/c-api/buffer.html
    image.data = bytes(image_local.image)
    # http://wiki.blender.org/index.php/Dev:Source/GameEngine/2.49/VideoTexture
    # http://www.blender.org/documentation/blender_python_api_2_57_release/bge.types.html#bge.types.KX_Camera.useViewport

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_name + "/image"):
            topic.publish(image)

    # sensor_msgs/CameraInfo [ http://ros.org/wiki/rviz/DisplayTypes/Camera ]
    # TODO fill this 3 parameters to get correcty image with stereo camera
    Tx = 0
    Ty = 0
    R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    intrinsic = component_instance.local_data['intrinsic_matrix']

    camera_info = CameraInfo()
    camera_info.header.stamp = image.header.stamp
    camera_info.header.seq = image.header.seq
    camera_info.header.frame_id = image.header.frame_id
    camera_info.height = image.height
    camera_info.width = image.width
    camera_info.distortion_model = 'plumb_bob'
    camera_info.K = [intrinsic[0][0], intrinsic[1][0], intrinsic[2][0], 
                     intrinsic[0][1], intrinsic[1][1], intrinsic[2][1], 
                     intrinsic[0][2], intrinsic[1][2], intrinsic[2][2]]
    camera_info.R = R
    camera_info.P = [intrinsic[0][0], intrinsic[1][0], intrinsic[2][0], Tx, 
                     intrinsic[0][1], intrinsic[1][1], intrinsic[2][1], Ty, 
                     intrinsic[0][2], intrinsic[1][2], intrinsic[2][2], 0]

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_name + "/camera_info"):
            topic.publish(camera_info)

    self._seq = self._seq + 1
