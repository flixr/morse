import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import mathutils
import morse.core.sensor
import copy

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class DeltaPoseClass(morse.core.sensor.MorseSensorClass):
    """ Delta pose Odometry sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.setLevel(logging.DEBUG)

        self.local_data['x'] = 0
        self.local_data['y'] = 0
        self.local_data['z'] = 0
        self.local_data['orientation'] = mathutils.Quaternion()

        self.previous_pose = copy.copy(self.position_3d)

        # get the transformation from robot to sensor frame
        (loc, rot, scale) = self.robot_parent.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
        logger.debug("body2sensor rotation RPY [% .3f % .3f % .3f]" % tuple(math.degrees(a) for a in rot.to_euler()))
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
