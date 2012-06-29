import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import mathutils
import morse.core.actuator

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class AsctecAttitudeCtrlActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller for Asctec Quadrotor Attitde control input

    This class will read desired attitude and thrust as input
    from an external middleware.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        #logger.setLevel(logging.DEBUG)

        # pitch input in radians
        self.local_data['pitch'] = 0.0
        # roll input in radians
        self.local_data['roll'] = 0.0
        #R/C Stick input: -20.47 .. 20.47 (0 = neutral)
        self.local_data['yaw'] = 0.0
        # Collective: 0 .. 1 (= 0 .. 100%)
        self.local_data['thrust'] = 0.0

        """
        control byte:
            bit 0: pitch control enabled
            bit 1: roll control enabled
            bit 2: yaw control enabled
            bit 3: thrust control enabled
          These bits can be used to only enable one axis at a time and thus to control
          the other axes manually. This usually helps a lot to set up and finetune
          controllers for each axis separately.
        """
        self.local_data['ctrl'] = 15

        self.add_property('_rp_pgain', 256, 'RollPitchPgain')
        self.add_property('_rp_dgain', 32, 'RollPitchDgain')
        self.add_property('_yaw_pgain', 8.0, 'YawPgain')
        self.add_property('_yaw_dgain', 4.0, 'YawDgain')
        self.add_property('_yaw_deadband', 2.5, 'YawDeadband')
        self.add_property('_max_yaw_rate', math.radians(60), 'MaxYawRate')
        self.add_property('_thrust_factor', 0.0255, 'ThrustFactor')

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardless of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        self.ticks = bge.logic.getLogicTicRate()
        logger.debug("logic tic rate: %d" % self.ticks)

        # Make new reference to the robot velocities (mathutils.Vector)
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity

        # get the robot inertia (list [ix, iy, iz])
        robot_inertia = self.robot_parent.blender_obj.localInertia
        self.inertia = mathutils.Vector(tuple(robot_inertia))
        logger.debug("robot inertia: (%.3f %.3f %.3f)" % tuple(self.inertia))

        # yaw setpoint in radians is just integrated from yaw rate input
        self.yaw_setpoint = 0.0

        self.prev_err = mathutils.Vector((0.0, 0.0, 0.0))

        logger.info('Component initialized')


    def default_action(self):
        """ Run attitude controller and apply resulting force and torque to the parent robot. """
        # Get the the parent robot
        robot = self.robot_parent

        yaw_in = self.local_data['yaw']
        if self.local_data['thrust'] > 0:
            # integrate desired yaw rate to angle setpoint
            if math.fabs(yaw_in) > self._yaw_deadband:
                yaw_rate = self._max_yaw_rate/20.47 * (yaw_in - math.copysign(self._yaw_deadband, yaw_in))
            else:
                yaw_rate = 0.0
            # yaw_rate and ya_setpoint in NED
            self.yaw_setpoint += yaw_rate / self.ticks
            # wrap angle
            while math.fabs(self.yaw_setpoint) > math.pi:
                self.yaw_setpoint -= math.copysign(2 * math.pi, self.yaw_setpoint)

            #logger.debug("yaw setpoint: %.3f" % (math.degrees(self.yaw_setpoint)))
            #logger.debug("yaw current: %.3f   setpoint: %.3f" % (-math.degrees(self.position_3d.yaw), math.degrees(self.yaw_setpoint)))

            # Compute errors
            #
            # e = att_sp - att = attitude error
            # current angles to horizontal plane in NED
            roll = self.position_3d.roll
            pitch = -self.position_3d.pitch
            yaw = -self.position_3d.yaw
            roll_err = self.local_data['roll'] - roll
            pitch_err = self.local_data['pitch'] - pitch
            yaw_err = self.yaw_setpoint - yaw
            # wrap yaw error
            while math.fabs(yaw_err) > math.pi:
                yaw_err -= math.copysign(2 * math.pi, yaw_err)
            err = mathutils.Vector((roll_err, pitch_err, yaw_err))
            #logger.debug("attitude error: (%.3f %.3f %.3f)" % (math.degrees(err[0]), math.degrees(err[1]), math.degrees(err[2])))

            # derivative
            we = (err - self.prev_err) * self.ticks
            #we = mathutils.Vector((0.0, 0.0, 0.0))
            #logger.debug("yaw rate error: %.3f" % (we[2]))

            kp = mathutils.Vector((self._rp_pgain, self._rp_pgain, self._yaw_pgain))
            kd = mathutils.Vector((self._rp_dgain, self._rp_pgain, self._yaw_dgain))

            #torque = self.inertia * (kp * err + kd * we)
            t = []
            for i in range(3):
                t.append(self.inertia[i] * (kp[i] * err[i] + kd[i] * we[i]))
            # convert to blender frame and scale with thrust
            torque = mathutils.Vector((t[0], -t[1], -t[2])) * self.local_data['thrust']
            #logger.debug("applied torques: (%.3f %.3f %.3f)" % (torque[0], torque[1], torque[2]))

            force = mathutils.Vector((0.0, 0.0, self.local_data['thrust'] / self._thrust_factor))
            #logger.debug("applied thrust force: %.3f" % (force[2]))

            self.prev_err = err.copy()
        else:
            force = mathutils.Vector((0.0, 0.0, 0.0))
            torque = mathutils.Vector((0.0, 0.0, 0.0))

        # directly apply local forces and torques to the blender object of the parent robot
        robot.blender_obj.applyForce(force, True)
        robot.blender_obj.applyTorque(torque, True)
