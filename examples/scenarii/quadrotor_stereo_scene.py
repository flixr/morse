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
imu_noise = True
with_object_detector = True
object_noise = False


# Robots
mav = Robot('quadrotor')
#mav.translate(x=-1.2483, y=1.7043, z=1.8106)
mav.translate(z=0.3)
#mav.rotate(z=math.radians(90))

if joystick_control:
    # Components
    motion = Actuator('quadrotor_attitude')
    motion.properties(YawPgain=10.0)
    motion.properties(YawDgain=6.0)
    motion.name = 'attitude'
    mav.append(motion)
    motion.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'read_attitude', 'morse/middleware/ros/read_asctec_attitude_ctrl'])

else:
    waypoint = Actuator('rotorcraft_waypoint')
    waypoint.properties(YawPgain=10.0)
    waypoint.properties(YawDgain=6.0)
    mav.append(waypoint)
    waypoint.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'read_pose', 'morse/middleware/ros/read_pose'])


imu = Sensor('imu')
imu.rotate(x=math.pi)
mav.append(imu)
#imu.configure_mw('ros')
imu.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_imu_mav', 'morse/middleware/ros/imu_mav'])
if imu_noise:
    imu.configure_modifier('foo', ['morse.modifiers.imu_noise.MorseIMUNoiseClass', 'noisify',
                                   {'gyro_std': 0.1, 'accel_std': 0.8}])

if with_height:
    height = Sensor('altitude')
    mav.append(height)
    height.properties(AltitudeOffset=0.226)
    height.frequency(10)
    height.name = 'height_to_footprint'
    height.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_height', 'morse/middleware/ros/laser_height'])

if with_object_detector:
    detector = Sensor('object_detector')
    mav.append(detector)
    detector.frequency(5)
    detector.properties(Target='PinkBox')
    detector.configure_mw('morse.middleware.ros_mw.rosclass', \
                          [MORSE_MIDDLEWARE_MODULE['ros'], 'post_pose', \
                           'morse/middleware/ros/pose', \
                           {'frame_id': '/detector'}])
    if object_noise:
        detector.configure_modifier('bar', ['morse.modifiers.pose_noise.MorsePoseNoiseClass',
                                            'noisify', {'pos_std': 0.1, 'rot_std': math.radians(10)}])

if with_pose:
    pose = Sensor('pose')
    pose.frequency(30)
    mav.append(pose)
    pose.configure_mw('morse.middleware.ros_mw.rosclass',
                      [MORSE_MIDDLEWARE_MODULE['ros'], 'post_tf',
                       'morse/middleware/ros/pose',
                       {'frame_id': '/world', 'child_frame_id': '/mav'}])

if with_stereo:
    # The STEREO UNIT, where the two cameras will be fixed
    stereo = Sensor('stereo_unit')
    stereo.translate(z= -0.03)
    stereo.rotate(y=math.radians(35))
    mav.append(stereo)

    # Left camera
    cameraL = Sensor('video_camera')
    cameraL.translate(x=0.1, y=0.05, z= -0.02)
    stereo.append(cameraL)
    cameraL.name = 'left'
    cameraL.properties(capturing=True)
    cameraL.properties(cam_width=376)
    cameraL.properties(cam_height=240)
    cameraL.properties(cam_focal=23)
    cameraL.properties(Vertical_Flip=True)
    cameraL.properties(cam_near=0.01)
    cameraL.frequency(5)
    cameraL.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_image_and_trigger', 'morse/middleware/ros/camera_with_trigger'])

    # Right camera
    right = Sensor('video_camera')
    right.translate(x=0.1, y= -0.05, z= -0.02)
    stereo.append(right)
    right.properties(capturing=True)
    right.properties(cam_width=376)
    right.properties(cam_height=240)
    right.properties(cam_focal=23)
    right.properties(Vertical_Flip=True)
    right.properties(cam_near=0.01)
    right.frequency(5)
    right.configure_mw('ros')

#env = Environment('indoors-1/boxes_dotted')
env = Environment('indoors-1/indoor-1_wp_marker')
#env = Environment('land-1/buildings_1')
env.show_framerate(True)
#env.show_physics(True)

env.create()
