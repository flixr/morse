from morse.builder.morsebuilder import *

import math

bpy.context.scene.game_settings.fps = 150
bpy.context.scene.game_settings.logic_step_max = 5
bpy.context.scene.game_settings.physics_step_max = 5

with_stereo = True

# Robots
Quadrotor = Robot('quadrotor')
#Quadrotor.translate(x=-1.2483, y=1.7043, z=1.8106)
Quadrotor.translate(z=0.3)
#Quadrotor.rotate(z=math.radians(90))
Quadrotor.name = 'mav'

# Components
motion = Actuator('quadrotor_attitude')
#motion.name = 'motion'
motion.properties(YawPgain = 10.0)
motion.properties(YawDgain = 6.0)
Quadrotor.append(motion)
motion.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'read_attitude', 'morse/middleware/ros/read_asctec_attitude_ctrl'])


imu = Sensor('imu')
imu.name = 'imu'
imu.rotate(x=math.pi)
Quadrotor.append(imu)
#imu.configure_mw('ros')
imu.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_imu_mav', 'morse/middleware/ros/imu_mav'])
#imu.configure_modifier('Noise')
#imu.configure_modifier([MORSE_MODIFIERS['IMUNoise'], 'noisify', 0.05, 0.5])
#imu.configure_modifier(['morse.modifiers.imu_noise.MorseIMUNoiseClass', 'noisify', 0.05, 0.5])

pose = Sensor('pose')
pose.frequency(30)
Quadrotor.append(pose)
pose.configure_mw('morse.middleware.ros_mw.rosclass', [MORSE_MIDDLEWARE_MODULE['ros'], 'post_tf', 'morse/middleware/ros/pose'])

if (with_stereo):
    # The STEREO UNIT, where the two cameras will be fixed
    Stereo = Sensor('stereo_unit')
    Stereo.translate(z=-0.03)
    Stereo.rotate(y=math.radians(35))
    Quadrotor.append(Stereo)
    
    # Left camera
    CameraL = Sensor('video_camera')
    CameraL.name = 'stereo/left'
    CameraL.translate(x=0.1, y=0.05, z=-0.02)
    Stereo.append(CameraL)
    CameraL.properties(capturing = True)
    CameraL.properties(cam_width = 376)
    CameraL.properties(cam_height = 240)
    CameraL.properties(cam_focal = 23)
    CameraL.properties(Vertical_Flip = True)
    CameraL.properties(cam_near = 0.01)
    CameraL.frequency(5)
    CameraL.configure_mw('morse.middleware.ros_mw.rosclass', ['morse.middleware.ros_mw.ROSClass', 'post_image_and_trigger', 'morse/middleware/ros/camera_with_trigger'])
    
    # Right camera
    CameraR = Sensor('video_camera')
    CameraR.name = 'stereo/right'
    CameraR.translate(x=0.1, y=-0.05, z=-0.02)
    Stereo.append(CameraR)
    CameraR.properties(capturing = True)
    CameraR.properties(cam_width = 376)
    CameraR.properties(cam_height = 240)
    CameraR.properties(cam_focal = 23)
    CameraR.properties(Vertical_Flip = True)
    CameraR.properties(cam_near = 0.01)
    CameraR.frequency(5)
    CameraR.configure_mw('ros')


env = Environment('indoors-1/indoor-1')
env.show_framerate(True)
#env.show_physics(True)

env.create()
