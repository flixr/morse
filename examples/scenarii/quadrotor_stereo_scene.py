from morse.builder import *

import math

# control = "joystick"
# control = "waypoint"
control = "velocity"
with_stereo = True
with_pose_rel = False
with_pose_tf = True
with_pose_gt = True
with_velocity_gt = True
with_height = False
with_imu = True
imu_noise = True
with_object_detector = False
object_noise = True


bpymorse.set_speed(fps=150, logic_step_max=5, physics_step_max=5)

# Robots
mav = Quadrotor()
# mav.translate(x=-1.2483, y=1.7043, z=1.8106)
mav.translate(z=0.3)
# mav.rotate(z=math.radians(90))
# mav.add_default_interface('ros')

if 'joystick' in control:
    # Components
    attitude = RotorcraftAttitude()
    attitude.properties(YawPgain=10.0)
    attitude.properties(YawDgain=6.0)
    mav.append(attitude)
    attitude.add_stream('ros', 'morse.middleware.ros.read_asctec_attitude_ctrl.AttitudeCtrlInputReader')

if 'velocity' in control:
    velocity = RotorcraftVelocity()
    mav.append(velocity)
    velocity.add_stream('ros')
    # velocity.add_stream('socket')

else:
    waypoint = RotorcraftWaypoint()
    waypoint.properties(YawPgain=10.0)
    waypoint.properties(YawDgain=6.0)
    waypoint.properties(MaxBankAngle=math.radians(10))
    mav.append(waypoint)
    waypoint.add_stream('ros')


if with_imu:
    imu = IMU()
    imu.rotate(x=math.pi)
    imu.name = 'imu'
    mav.append(imu)
    # imu.add_stream('ros')
    imu.add_stream('ros', 'morse.middleware.ros.imu_mav.ImuMavPublisher')
    if imu_noise:
        imu.alter('Noise', gyro_std=0.03, accel_std=0.2)

if with_height:
    height = Altitude()
    height.name = 'height'
    mav.append(height)
    height.properties(AltitudeOffset=0.226)
    height.frequency(10)
    height.name = 'height_to_footprint'
    height.add_stream('ros', 'post_height', 'morse/middleware/ros/laser_height')

if with_object_detector:
    # feature detector for visualization in Rviz
    viz = ObjectDetector()
    viz.translate(x=0.0704, y=0.0500, z= -0.1037)
    viz.rotate(x= -math.radians(125), z= -math.pi / 2)
    mav.append(viz)
    viz.name = 'feature_viz'
    viz.frequency(2)
    viz.properties(Target='PinkBox')
    viz.properties(DetectionDistance=2)
    viz.add_stream('ros', 'post_tf', 'morse/middleware/ros/pose',
                           {'frame_id': '/cam_left_gt', 'child_frame_id': '/feature_viz'})
    # actual detector with covariance
    detector = ObjectDetector()
    detector.translate(x=0.0704, y=0.0500, z= -0.1037)
    detector.rotate(x= -math.radians(125), z= -math.pi / 2)
    mav.append(detector)
    detector.name = 'feature_pose'
    detector.frequency(3)
    detector.properties(Target='feature')
    detector.properties(PosStdZ=0.01)
    detector.properties(RotStd=math.radians(1))
    detector.properties(DetectionDistance=2)
    detector.add_stream('ros', 'post_pose_with_covariance_trigger', 'morse/middleware/ros/pose_trigger',
                           {'frame_id': '/cam_left', 'child_frame_id': '/feature'})
    if object_noise:
        detector.alter('', 'morse.modifiers.pose_noise.PoseNoiseModifier', pos_std=0.01, rot_std=math.radians(1))

if with_pose_tf:
    pose_tf = Pose()
    pose_tf.frequency(30)
    mav.append(pose_tf)
    pose_tf.add_stream('ros', 'morse.middleware.ros.pose.TFPublisher', frame_id='/blender', child_frame_id='/mav_gt')

if with_pose_gt:
    pose_gt = Pose()
    pose_gt.frequency(30)
    pose_gt.rotate(x=math.pi)
    mav.append(pose_gt)
    # pose_gt.add_stream('ros', 'morse.middleware.ros.pose.PoseStampedPublisher', frame_id='/blender')
    pose_gt.add_stream('ros', frame_id='/blender')

if with_velocity_gt:
    velocity_gt = Velocity()
    velocity_gt.frequency(30)
    mav.append(velocity_gt)
    velocity_gt.add_stream('ros', frame_id='/blender')

if with_stereo:
    # The STEREO UNIT, where the two cameras will be fixed
    stereo = StereoUnit()
    stereo.translate(z= -0.03)
    stereo.rotate(y=math.radians(35))
    mav.append(stereo)

    # Left camera
    cameraL = VideoCamera()
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
    cameraL.add_stream('ros', 'morse.middleware.ros.video_camera.VideoCameraPublisher')
    #cameraL.add_stream('ros', 'morse.middleware.ros.gpio_event.GpioEventPublisher', topic='/cam_trigger')

    # Right camera
    right = VideoCamera()
    right.translate(x=0.1, y= -0.05, z= -0.02)
    right.name = 'right'
    stereo.append(right)
    right.properties(capturing=True)
    right.properties(cam_width=376)
    right.properties(cam_height=240)
    right.properties(cam_focal=23)
    right.properties(Vertical_Flip=True)
    right.properties(cam_near=0.01)
    right.frequency(5)
    right.add_stream('ros')

if with_pose_rel:
    poseRel = DeltaPose()
    poseRel.translate(x=0.0704, y=0.05, z= -0.1037)
    poseRel.rotate(x= -math.radians(125), z= -math.radians(90))
    poseRel.frequency(5)
    mav.append(poseRel)
    poseRel.add_stream('ros', 'morse.middleware.ros.visual_odometry.VisualOdometryPublisher')
    # poseRel.add_stream('ros', 'morse.middleware.ros.gpio_event.GpioEventPublisher', topic='/cam_trigger')
    poseRel.alter('', 'morse.modifiers.pose_noise.PoseNoiseModifier', pos_std={'x':0.01, 'y':0.01, 'z':0.1}, rot_std=math.radians(1))



env = Environment('indoors-1/indoor-1')
# env = Environment('indoors-1/indoor-1_feature')
# env = Environment('land-1/buildings_1')
env.show_framerate(True)
# env.show_physics(True)
env.create()
