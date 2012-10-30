from morse.builder.morsebuilder import *

import math

bpy.context.scene.game_settings.fps = 150
bpy.context.scene.game_settings.logic_step_max = 5
bpy.context.scene.game_settings.physics_step_max = 5
bpy.context.scene.game_settings.physics_step_sub = 2

joystick_control = False
with_stereo = False
with_pose_rel = True
with_pose_tf = True
with_pose = True
with_velocity = True
with_height = True
imu_noise = True
with_object_detector = True
object_noise = True


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
    motion.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'read_attitude', 'morse/middleware/ros/read_asctec_attitude_ctrl'])

else:
    waypoint = Actuator('rotorcraft_waypoint')
    waypoint.properties(YawPgain=10.0)
    waypoint.properties(YawDgain=6.0)
    waypoint.properties(MaxBankAngle=math.radians(10))
    mav.append(waypoint)
    waypoint.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'read_pose', 'morse/middleware/ros/read_pose'])


imu = Sensor('imu')
imu.rotate(x=math.pi)
mav.append(imu)
#imu.configure_mw('ros')
imu.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'post_imu_mav', 'morse/middleware/ros/imu_mav'])
if imu_noise:
    imu.configure_modifier('foo', ['morse.modifiers.imu_noise.MorseIMUNoiseClass', 'noisify',
                                   {'gyro_std': 0.03, 'accel_std': 0.2}])

if with_height:
    height = Sensor('altitude')
    mav.append(height)
    height.properties(AltitudeOffset=0.226)
    height.frequency(10)
    height.name = 'height_to_footprint'
    height.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'post_height', 'morse/middleware/ros/laser_height'])

if with_object_detector:
    # feature detector for visualization in Rviz
    viz = Sensor('object_detector')
    viz.translate(x=0.0704, y=0.0500, z= -0.1037)
    viz.rotate(x= -math.radians(125), z= -math.pi / 2)
    mav.append(viz)
    viz.name = 'feature_viz'
    viz.frequency(2)
    viz.properties(Target='PinkBox')
    viz.properties(DetectionDistance=2)
    viz.configure_mw('ros', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_tf',
                           'morse/middleware/ros/pose',
                           {'frame_id': '/cam_left_gt', 'child_frame_id': '/feature_viz'}])
    # actual detector with covariance
    detector = Sensor('object_detector')
    detector.translate(x=0.0704, y=0.0500, z= -0.1037)
    detector.rotate(x= -math.radians(125), z= -math.pi / 2)
    mav.append(detector)
    detector.name = 'feature_pose'
    detector.frequency(3)
    detector.properties(Target='feature')
    detector.properties(PosStdZ=0.01)
    detector.properties(RotStd=math.radians(1))
    detector.properties(DetectionDistance=2)
    detector.configure_mw('ros', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_pose_with_covariance_trigger',
                           'morse/middleware/ros/pose_trigger',
                           {'frame_id': '/cam_left', 'child_frame_id': '/feature'}])
    if object_noise:
        detector.configure_modifier('bar', ['morse.modifiers.pose_noise.MorsePoseNoiseClass',
                                            'noisify', {'pos_std': [0.01, 0.01, 0.01], 'rot_std': [math.radians(1)] * 3}])

if with_pose_tf:
    pose_tf = Sensor('pose')
    pose_tf.frequency(30)
    mav.append(pose_tf)
    pose_tf.configure_mw('ros', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_tf',
                         'morse/middleware/ros/pose',
                         {'frame_id': '/blender', 'child_frame_id': '/mav_gt'}])

if with_pose:
    pose_gt = Sensor('pose')
    pose_gt.frequency(30)
    pose_gt.rotate(x=math.pi)
    mav.append(pose_gt)
    pose_gt.configure_mw('ros', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_pose',
                         'morse/middleware/ros/pose',
                         {'frame_id': '/blender'}])

if with_velocity:
    velocity_gt = Sensor('velocity')
    velocity_gt.frequency(30)
    mav.append(velocity_gt)
    velocity_gt.configure_mw('ros', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_twist',
                         'morse/middleware/ros/velocity',
                         {'frame_id': '/blender'}])

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
    # opening angle: arctan(cam_focal/(32/2))
    # cam_focal of 23 -> 55deg opening angle
    cameraL.properties(cam_focal=23)
    cameraL.properties(Vertical_Flip=True)
    cameraL.properties(cam_near=0.01)
    cameraL.frequency(5)
    cameraL.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'post_image_and_trigger', 'morse/middleware/ros/camera_with_trigger'])

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

if with_pose_rel:
    poseRel = Sensor('delta_pose')
    poseRel.translate(x=0.0704, y=0.05, z= -0.1037)
    poseRel.rotate(x= -math.radians(125), z= -math.radians(90))
    poseRel.frequency(5)
    mav.append(poseRel)
    poseRel.configure_mw('ros', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_visual_odometry', 'morse/middleware/ros/visual_odometry'])
    poseRel.configure_modifier('bar', ['morse.modifiers.pose_noise.MorsePoseNoiseClass',
                                       'noisify', {'pos_std': [0.01, 0.01, 0.01], 'rot_std': [math.radians(1)] * 3}])

#env = Environment('indoors-1/boxes_dotted')
env = Environment('indoors-1/indoor-1_feature')
#env = Environment('land-1/buildings_1')
env.show_framerate(True)
#env.show_physics(True)

env.create()
