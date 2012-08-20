import logging; logger = logging.getLogger("morse." + __name__)

import math
import bge
import mathutils
import morse.core.actuator
from morse.core.services import service
from morse.core.services import async_service
from morse.core.services import interruptible
from morse.core import status

class RotorcraftWaypointActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Waypoint motion controller

    This controller will receive a 3D destination point and heading
    and make the robot move to that location by changing attitude.
    This controller is meant for rotorcrafts like quadrotors.
    """

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.setLevel(logging.INFO)

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardless of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        self.ticks = bge.logic.getLogicTicRate()

        # The actual frequency at which the sensor action is called
        # When a delay of the sensor is set via frequency,
        # the action is not called for every logic tick.
        # frequency of the game sensor specifies how many actions are skipped
        # e.g. game sensor freq = 0 -> sensor runs at full logic rate
        self.freq = self.ticks / (self.blender_obj.sensors[0].frequency + 1)
        logger.info("Running waypoint actutator at %.2f Hz", self.freq)


        self._destination = self.robot_parent.blender_obj.worldPosition
        self._wp_object = None

        self.add_property('_h_pgain', math.radians(6), 'HorizontalPgain')
        self.add_property('_h_dgain', math.radians(8), 'HorizontalDgain')
        self.add_property('_v_pgain', 8, 'VerticalPgain')
        self.add_property('_v_dgain', 8, 'VerticalDgain')
        self.add_property('_yaw_pgain', 17.8, 'YawPgain')
        self.add_property('_yaw_dgain', 3.4, 'YawDgain')
        self.add_property('_rp_pgain', 9.7, 'RollPitchPgain')
        self.add_property('_rp_dgain', 2, 'RollPitchDgain')
        self.add_property('_max_bank_angle', math.radians(30), 'MaxBankAngle')
        self.add_property('_target', 'wp_target', 'Target')

        self.local_data['x'] = self._destination[0]
        self.local_data['y'] = self._destination[1]
        self.local_data['z'] = self._destination[2]
        self.local_data['yaw'] = self.robot_parent.position_3d.yaw
        # Waypoint tolerance (in meters)
        self.local_data['tolerance'] = 0.2

        logger.info("inital wp: (%.3f %.3f %.3f)", self._destination[0], self._destination[0], self._destination[0])
        self._pos_initalized = False

        # Make new reference to the robot velocities (mathutils.Vector)
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity

        # get the robot inertia (list [ix, iy, iz])
        robot_inertia = self.robot_parent.blender_obj.localInertia
        self.inertia = mathutils.Vector(tuple(robot_inertia))
        logger.info("robot inertia: (%.3f %.3f %.3f)" % tuple(self.inertia))

        self.nominal_thrust = self.robot_parent.blender_obj.mass * 9.81
        logger.info("nominal thrust: %.3f", self.nominal_thrust)
        self._attitude_compensation_limit = math.cos(self._max_bank_angle) ** 2

        # current attitude setpoints in radians
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0

        self.thrust = 0.0

        #previous attitude error
        self.prev_err = mathutils.Vector((0.0, 0.0, 0.0))

        # Identify an object as the target of the motion
        try:
            wp_name = self._target
            if wp_name != '':
                scene = bge.logic.getCurrentScene()
                self._wp_object = scene.objects[wp_name]
                logger.info("Using object '%s' to indicate motion target", wp_name)
        except KeyError as detail:
            self._wp_object = None
            logger.info("Not using a target object")


        logger.info('Component initialized')


    @service
    def setdest(self, x, y, z, yaw, tolerance=0.2):
        """ Set a new waypoint and returns.

        The robot will try to go to this position, but the service 
        caller has no mean to know when the destination is reached
        or if it failed.

        In most cases, the asynchronous service 'goto' should be 
        preferred.

        Returns always True (if the robot is already moving, the
        previous target is replaced by the new one) except if
        the destination is already reached. In this case, returns
        False.
        """

        distance, gv, lv = self.robot_parent.blender_obj.getVectTo([x, y, z])
        if distance - tolerance <= 0:
            logger.info("Robot already at destination (distance = {})."
                    " I do not set a new destination.".format(distance))
            return False

        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['yaw'] = yaw
        self.local_data['tolerance'] = tolerance

        return True


    @interruptible
    @async_service
    def goto(self, x, y, z, yaw, tolerance=0.2):
        """ Go to a new destination.

        The service returns when the destination is reached within
        the provided tolerance bounds.
        """
        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['yaw'] = yaw
        self.local_data['tolerance'] = tolerance


    @service
    #@async_service
    def stop(self):
        """ Interrupt the movement of the robot """
        #self.local_data['x'] = self.blender_obj.worldPosition[0]
        #self.local_data['y'] = self.blender_obj.worldPosition[1]
        #self.local_data['z'] = self.blender_obj.worldPosition[2]
        #self.local_data['tolerance'] = 0.5

        # Set the status of the robot
        self.robot_parent.move_status = "Stop"

        return self.robot_parent.move_status

    @service
    def resume(self):
        """ Restore the previous speed and keep going towards the waypoint """
        self.local_data['speed'] = self._previous_speed
        self._previous_speed = 0

        # Set the status of the robot
        self.robot_parent.move_status = "Transit"

        return self.robot_parent.move_status


    @service
    def get_status(self):
        """ Return the current status (Transit, Arrived or Stop) """
        return self.robot_parent.move_status


    def default_action(self):
        """ Move the object towards the destination. """
        robot = self.robot_parent

        if self._pos_initalized == False:
            self._destination = self.robot_parent.blender_obj.worldPosition
            self._pos_initalized = True

        self._destination = mathutils.Vector((self.local_data['x'], self.local_data['y'], self.local_data['z']))

        #logger.debug("Robot %s move status: '%s'", robot.blender_obj.name, robot.move_status)
        # Place the target marker where the robot should go
        if self._wp_object:
            self._wp_object.position = self._destination
            #self._wp_opject.rotation.yaw = self.local_data['yaw']

        # current angles to horizontal plane
        roll = self.position_3d.roll
        pitch = self.position_3d.pitch
        yaw = self.position_3d.yaw

        # current position and velocity of robot 
        pos_blender = robot.blender_obj.worldPosition
        #pos_ned = mathutils.Vector((pos_blender[0], -pos_blender[1], -pos_blender[2]))
        vel_blender = robot.blender_obj.worldLinearVelocity
        #vel_ned = mathutils.Vector((vel_blender[0], -vel_blender[1], -vel_blender[2]))

        pos_error = self._destination - pos_blender
        # zero velocity setpoint for now
        vel_error = -vel_blender

        logger.debug("pos current: (%.3f %.3f %.3f) setpoint: (%.3f %.3f %.3f)", pos_blender[0], pos_blender[1], pos_blender[2],
                    self._destination[0], self._destination[1], self._destination[2])
        #logger.debug("velocity: (%.3f %.3f %.3f)", vel[0], vel[1], vel[2])

        # simple PD controller on horizontal position
        command_world_x = self._h_pgain * pos_error[0] + self._h_dgain * vel_error[0]
        command_world_y = self._h_pgain * pos_error[1] + self._h_dgain * vel_error[1]

        # setpoints in body frame (NED)
        self.roll_setpoint = math.sin(yaw) * command_world_x - math.cos(yaw) * command_world_y
        self.pitch_setpoint = math.cos(yaw) * command_world_x + math.sin(yaw) * command_world_y
        self.yaw_setpoint = self.local_data['yaw']

        # saturate max roll/pitch angles
        if math.fabs(self.roll_setpoint) > self._max_bank_angle:
            self.roll_setpoint = math.copysign(self._max_bank_angle, self.roll_setpoint)
        if math.fabs(self.pitch_setpoint) > self._max_bank_angle:
            self.pitch_setpoint = math.copysign(self._max_bank_angle, self.pitch_setpoint)

        # wrap yaw angle
        while math.fabs(self.yaw_setpoint) > math.pi:
            self.yaw_setpoint -= math.copysign(2 * math.pi, self.yaw_setpoint)

        logger.debug("roll  current: %.3f   setpoint: %.3f", math.degrees(roll), math.degrees(self.roll_setpoint))
        logger.debug("pitch current: %.3f   setpoint: %.3f", math.degrees(pitch), math.degrees(self.pitch_setpoint))
        logger.debug("yaw   current: %.3f   setpoint: %.3f", math.degrees(yaw), math.degrees(self.yaw_setpoint))


        # compute thrust
        # nominal command to keep altitude (feed forward)
        thrust_attitude_compensation = max(self._attitude_compensation_limit, math.cos(roll) * math.cos(pitch))
        thrust_ff = self.nominal_thrust / thrust_attitude_compensation
        # feedback to correct altitude
        thrust_fb = self._v_pgain * pos_error[2] + self._v_dgain * vel_error[2]
        self.thrust = thrust_ff + thrust_fb


        # Compute attitude errors
        #
        # e = att_sp - att = attitude error
        roll_err = self.roll_setpoint - roll
        pitch_err = self.pitch_setpoint - pitch
        yaw_err = self.yaw_setpoint - yaw
        # wrap yaw error
        while math.fabs(yaw_err) > math.pi:
            yaw_err -= math.copysign(2 * math.pi, yaw_err)
        err = mathutils.Vector((roll_err, pitch_err, yaw_err))
        logger.debug("attitude error: (%.3f %.3f %.3f)", math.degrees(err[0]), math.degrees(err[1]), math.degrees(err[2]))

        # derivative
        we = (err - self.prev_err) * self.freq
        #we = mathutils.Vector((0.0, 0.0, 0.0))
        #logger.debug("yaw rate error: %.3f", we[2])

        kp = mathutils.Vector((self._rp_pgain, self._rp_pgain, self._yaw_pgain))
        kd = mathutils.Vector((self._rp_dgain, self._rp_pgain, self._yaw_dgain))

        #torque = self.inertia * (kp * err + kd * we)
        t = []
        for i in range(3):
            t.append(self.inertia[i] * (kp[i] * err[i] + kd[i] * we[i]))
        # scale with thrust
        torque = mathutils.Vector((t[0], t[1], t[2])) # * self.thrust / self.nominal_thrust
        logger.debug("applied torques: (%.3f %.3f %.3f)", torque[0], torque[1], torque[2])

        force = mathutils.Vector((0.0, 0.0, self.thrust))
        logger.debug("applied thrust force: %.3f", force[2])

        self.prev_err = err.copy()

        # directly apply local forces and torques to the blender object of the parent robot
        robot.blender_obj.applyForce(force, True)
        robot.blender_obj.applyTorque(torque, True)


        # Vectors returned are already normalized
        wp_distance, global_vector, local_vector = self.blender_obj.getVectTo(self._destination)

        #logger.debug("GOT DISTANCE: xyz: %.4f", wp_distance)

        # If the target has been reached, change the status
        if wp_distance - self.local_data['tolerance'] <= 0:
            robot.move_status = "Arrived"

            #Do we have a running request? if yes, notify the completion
            self.completed(status.SUCCESS, robot.move_status)

            #logger.debug("TARGET REACHED")
            #logger.debug("Robot %s move status: '%s'", robot.blender_obj.name, robot.move_status)

        else:
            robot.move_status = "Transit"
