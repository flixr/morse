import logging; logger = logging.getLogger("morse." + __name__)
from math import degrees
import mathutils
import morse.core.sensor
import copy
from morse.helpers.components import add_data

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class DeltaPose(morse.core.sensor.Sensor):
    """ Delta pose Odometry sensor """

    _name = "Delta Pose"
    _short_desc = "Delta pose between two measurements."

    # Sensor.add_property('Speed', 1.0, 'speed of each joint, in rad/s')

    add_data('x', 0.0, "float", "delta of X coordinate of the sensor")
    add_data('y', 0.0, "float", "delta of Y coordinate of the sensor")
    add_data('z', 0.0, "float", "delta of Z coordinate of the sensor")
    add_data('orientation', mathutils.Quaternion, "quaternion", "delta orientation")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.setLevel(logging.DEBUG)

        self.previous_pose = copy.copy(self.position_3d)

        # get the transformation from robot to sensor frame
        (loc, rot, _) = self.robot_parent.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
        logger.debug("body2sensor rotation RPY [% .3f % .3f % .3f]" % tuple(degrees(a) for a in rot.to_euler()))
        logger.debug("body2sensor translation [% .4f % .4f % .4f]" % tuple(loc))

        logger.info("DeltaPose Component initialized, runs at %.2f Hz ", self.frequency)


    def default_action(self):
        """
        Get the relative pose between two time steps
        """

        tf_prev2now = self.previous_pose.transformation3d_with(self.position_3d)

        self.previous_pose = copy.copy(self.position_3d)

        # Store the important data
        self.local_data['x'] = tf_prev2now.x
        self.local_data['y'] = tf_prev2now.y
        self.local_data['z'] = tf_prev2now.z
        self.local_data['orientation'] = tf_prev2now.rotation
