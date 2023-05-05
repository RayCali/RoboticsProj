# NOTES
# token for pushing and pulling
#### The username
robotgrumpy
#### The token
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

 rostopic pub /motor/duty_cycles robp_msgs/DutyCycles
 ~/dd2419_ws/src/robp_robot/robp_phidgets/launch
# phidgetslaunch directory
 /dd2419_ws/src/robp_robot/robp_phidgets/launch$ 
rostopic pub /cmd_vel geometry_msgs/Twist 
# builds and sources put into bashrc
alias cb="cd ~/dd2419_ws;catkin build;cd -;source ~/dd2419_ws/devel/setup.bash"
# launch camera
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
roslaunch realsense2_camera rs_camera.launch align_depth:=True
# launch everything else including odometry and cartesian controllerS
roslaunch robp_phidgets phidgets.launch
# usb_cam name 
for some reason it changes name for the usb_cam between /dev/video6 (when launching it seperatly) 
and /dev/video0 (when launching it together with everything using phidgets.launch)
#topic for rgbd camera
/camera/color/image_raw
# record a rosbag 
rosbag record /motor/duty_cycles /usb_cam/image_raw /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/color/points /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /tf /tf_static (name.bag)


# launch for playing rosbags
roslaunch robp_phidgets phidgetsrosplayer.launch
#play a rosbag 
 rosbag play (name.bag)

# How to launch Grumpy
1. Roscore
2. Check what the robot camera is called
    1. ls -ltrh /dev/video*
    2. By trial and error, check topic the camera publishes to and then add the correct /dev/camera# into 
         ~/dd2419_ws/src/usb_cam/launch/usb_cam-test.launch
    3. It should be either *video0* or *video6
3. Realsense
   1. roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
4. Phidgets
      1. roslaunch robp_phidgets phidgets.launch
 
5. If you wish to record, then launch
    rosbag record /motor/duty_cycles /usb_cam/image_raw /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/color/points /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /tf /tf_static (name.bag)

6. If you wish to play back
   roslaunch robp_phidgets phidgetsrosplayer.launch 

# How to use RVIZ on another computer
## STATIC IP
While the robot is on the IoT network, it should have a static ip. The last known is:
192.168.128.111
## On the robot
1. find the *robot inet address* by using the ifconfig command. You know it is the right command if you are able to ssh into the robot with this address.

2. in the .bashrc file of the robot

    a. set "export ROS_HOSTNAME= *robot inet address*"
    b. set "export ROS_MASTER_URI=http://*robot inet address*:11311"

## On the SSD
1. set "export ROS_MASTER_URI=http://*robot inet address*:11311"
2. find the *ssd inet address*
3. set "export ROS_IP=*ssd inet address*"
4. launch rviz on the SSD with a config file using "rosrun rviz rviz -d (path_to_config_file)"
   (the config file can be found on the google drive)
## How to fix Rosbag
1. rosbag fix (bag_name) (new_bagname) if it asks for it
2. rosbag reindex (bag_name)

# Robot arm
## Launching the arm
1. In ~/.bashrc uncomment the line with "export ROS_IP=10.0.0.1" to enable publishing to the arm
2. Start a new terminator/terminal and run roscore
3. Switch on the raspberry Pi and wait for some time (until the arm moves into its home position and greets you with a **beep**)
4. To verify that the arm is running, run rostopic list (you should see a list of arm topics)
5. To move seperate joints from terminal run *rostopic pub /joint1_controller/command_duration hiwonder_servo_msgs/CommandDuration "data: 0.0 duration: 0.0"*
   - You can also publish to /jointx_controller/command (where x is joint number)
   - The published value is the joint position

## Other info
- home position: [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0, -1.7802358066666664] (last joint is gripper)
- the action server for following trajectories is /arm_controller/follow_joint_trajectory


# Glitches
## catkin packages being unbuilt

# Services, clients and providers
doExplore, mapping_and_planning -> brain
