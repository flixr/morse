import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor
from math import degrees

class VelocityClass(morse.core.sensor.MorseSensorClass):
    """ Robot velocity sensor, including linear and angular velocity """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.local_data['linear_velocity'] = [0.0, 0.0, 0.0]
        self.local_data['angular_velocity'] = [0.0, 0.0, 0.0]
        self.local_data['world_linear_velocity'] = [0.0, 0.0, 0.0]

        # The robot needs a physics controller!
        # Since the sensor does not have physics
        if not bool(self.robot_parent.blender_obj.getPhysicsId()):
            logger.error("The robot doesn't have a physics controller!")

        # make new references to the robot velocities and use those.
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity
        self.robot_v = self.robot_parent.blender_obj.localLinearVelocity
        self.robot_world_v = self.robot_parent.blender_obj.localLinearVelocity

        # get the transformation from robot to sensor frame
        (loc, rot, scale) = self.robot_parent.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
        logger.debug("body2sensor rotation RPY [% .3f % .3f % .3f]" % tuple(degrees(a) for a in rot.to_euler()))
        # store body to imu rotation and translation
        self.rot_b2s = rot

        # reference for rotating world frame to imu frame
        self.rot_w2s = self.blender_obj.worldOrientation

        logger.info("Component initialized, runs at %.2f Hz", self.frequency)


    def default_action(self):
        """ Get the linear and angular velocity of the blender object. """

        # rot_b2i rotates body frame to imu frame
        # take the inverse rotation to transform a vector from body to sensor
        rates = self.rot_b2s.inverted() * self.robot_w
        vel = self.rot_b2s.inverted() * self.robot_v
        world_vel = self.rot_w2s.inverted() * self.robot_world_v

        # Store the important data
        self.local_data['linear_velocity'] = vel
        self.local_data['angular_velocity'] = rates
        self.local_data['world_linear_velocity'] = world_vel
