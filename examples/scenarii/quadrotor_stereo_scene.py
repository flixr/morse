from morse.builder.morsebuilder import *

import math

bpy.context.scene.game_settings.fps = 30
bpy.context.scene.game_settings.logic_step_max = 5
bpy.context.scene.game_settings.physics_step_max = 5
bpy.context.scene.game_settings.physics_step_sub = 2

with_stereo = True
with_pose = True
with_height = True
joystick_control = False
imu_noise = False
with_object_detector = True


# Robots
Quadrotor = Robot('quadrotor')
#Quadrotor.translate(x=-1.2483, y=1.7043, z=1.8106)
Quadrotor.translate(z=0.3)
#Quadrotor.rotate(z=math.radians(90))
Quadrotor.name = 'mav'

if joystick_control:
    # Components
    motion = Actuator('quadrotor_attitude')
    #motion.name = 'motion'
    motion.properties(YawPgain=10.0)
    motion.properties(YawDgain=6.0)
    Quadrotor.append(motion)
    motion.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'read_attitude', 'morse/middleware/ros/read_asctec_attitude_ctrl'])

else:
    waypoint = Actuator('rotorcraft_waypoint')
    waypoint.properties(YawPgain=10.0)
    waypoint.properties(YawDgain=6.0)
    Quadrotor.append(waypoint)
    waypoint.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'read_pose', 'morse/middleware/ros/read_pose'])


imu = Sensor('imu')
imu.name = 'imu'
imu.rotate(x=math.pi)
Quadrotor.append(imu)
#imu.configure_mw('ros')
imu.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_imu_mav', 'morse/middleware/ros/imu_mav'])
if imu_noise:
    imu.configure_modifier('foobar', ['morse.modifiers.imu_noise.MorseIMUNoiseClass', 'noisify', {'gyro_std': 0.1, 'accel_std': 0.8}])

if with_height:
    height = Sensor('altitude')
    height.name = 'height_to_footprint'
    Quadrotor.append(height)
    height.properties(AltitudeOffset=0.226)
    height.frequency(10)
    height.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_height', 'morse/middleware/ros/laser_height'])

if with_object_detector:
    detector = Sensor('object_detector')
    detector.name = 'detector'
    Quadrotor.append(detector)
    detector.frequency(5)
    detector.properties(Target='PinkBox')
    detector.configure_mw('morse.middleware.ros_mw.rosclass', \
                          [MORSE_MIDDLEWARE_MODULE['ros'], 'post_pose', \
                           'morse/middleware/ros/pose', \
                           {'frame_id': '/detector'}])

if with_pose:
    pose = Sensor('pose')
    pose.frequency(30)
    Quadrotor.append(pose)
    pose.configure_mw('morse.middleware.ros_mw.rosclass', \
                      [MORSE_MIDDLEWARE_MODULE['ros'], 'post_tf', \
                       'morse/middleware/ros/pose', \
                       {'frame_id': '/world', 'child_frame_id': '/mav'}])

if with_stereo:
    # The STEREO UNIT, where the two cameras will be fixed
    Stereo = Sensor('stereo_unit')
    Stereo.translate(z= -0.03)
    Stereo.rotate(y=math.radians(35))
    Quadrotor.append(Stereo)

    # Left camera
    CameraL = Sensor('video_camera')
    CameraL.name = 'stereo/left'
    CameraL.translate(x=0.1, y=0.05, z= -0.02)
    Stereo.append(CameraL)
    CameraL.properties(capturing=True)
    CameraL.properties(cam_width=376)
    CameraL.properties(cam_height=240)
    CameraL.properties(cam_focal=23)
    CameraL.properties(Vertical_Flip=True)
    CameraL.properties(cam_near=0.01)
    CameraL.frequency(5)
    CameraL.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_image_and_trigger', 'morse/middleware/ros/camera_with_trigger'])

    # Right camera
    CameraR = Sensor('video_camera')
    CameraR.name = 'stereo/right'
    CameraR.translate(x=0.1, y= -0.05, z= -0.02)
    Stereo.append(CameraR)
    CameraR.properties(capturing=True)
    CameraR.properties(cam_width=376)
    CameraR.properties(cam_height=240)
    CameraR.properties(cam_focal=23)
    CameraR.properties(Vertical_Flip=True)
    CameraR.properties(cam_near=0.01)
    CameraR.frequency(5)
    CameraR.configure_mw('ros')

#env = Environment('indoors-1/boxes_dotted')
env = Environment('indoors-1/indoor-1_wp_marker')
#env = Environment('land-1/buildings_1')
env.show_framerate(True)
#env.show_physics(True)

env.create()
