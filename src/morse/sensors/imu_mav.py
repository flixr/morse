import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import mathutils
import morse.core.sensor

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class ImuMavClass(morse.core.sensor.MorseSensorClass):
    """ IMU sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardless of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        self.ticks = bge.logic.getLogicTicRate()

        # Make new reference to the robot velocities
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity
        self.robot_vel = self.robot_parent.blender_obj.worldLinearVelocity

        # previous linear velocity
        self.plv = mathutils.Vector((0.0, 0.0, 0.0))

        # get gravity from scene?
        #g = bpy.data.scenes[0].game_settings.physics_gravity
        g = 9.81
        self.gravity = mathutils.Vector((0.0, 0.0, g))

        # get the transformation from robot to sensor frame
        (loc, rot, scale) = self.robot_parent.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
        #logger.debug("rotation [%.4f %.4f %.4f]" % tuple(math.degrees(a) for a in rot.to_euler()))
        self.rot_b2i = rot

        self.local_data['angular_velocity'] = [0.0, 0.0, 0.0]
        self.local_data['linear_acceleration'] = [0.0, 0.0, 0.0]
        self.local_data['magnetometer'] = [0.0, 0.0, 0.0]
        self.local_data['baro'] = 0.0

        logger.info('Component initialized')


    def default_action(self):
        """ Get the speed and acceleration of the robot and transform it into the imu frame """

        #logger.debug("rates in robot frame (%.4f, %.4f, %.4f)" % (self.robot_w[0], self.robot_w[1], self.robot_w[2]))

        rates = self.rot_b2i * self.robot_w
        #logger.debug("RATES in imu frame (%.4f, %.4f, %.4f)" % (rates[0], rates[1], rates[2]))

        rot_w2i = self.blender_obj.worldOrientation
        # rotate differentiated velocity to imu frame
        dv_imu = rot_w2i * (self.robot_vel - self.plv) * self.ticks
        #logger.debug("velocity_dot in imu frame (%.4f, %.4f, %.4f)" % (dv_imu[0], dv_imu[1], dv_imu[2]))
        accel_meas = dv_imu + rot_w2i * self.gravity

        # save velocity for next step
        self.plv = self.robot_vel.copy()

        # Store the important data
        self.local_data['angular_velocity'][0] = rates[0]
        self.local_data['angular_velocity'][1] = rates[1]
        self.local_data['angular_velocity'][2] = rates[2]

        self.local_data['linear_acceleration'][0] = accel_meas[0]
        self.local_data['linear_acceleration'][1] = accel_meas[1]
        self.local_data['linear_acceleration'][2] = accel_meas[2]
