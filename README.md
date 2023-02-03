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
# launch odometry and cartesian controller
roslaunch robp_phidgets phidgets.launch
# usb_cam name 
for some reason it changes name for the usb_cam between /dev/video6 (when launching it seperatly) 
and /dev/video0 (when launching it together with everything using phidgets.launch)
#topic for rgbd camera
/camera/color/image_raw
# record a rosbag 
rosbag record /motor/duty_cycles /usb_cam/image_raw /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/color/points /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /tf /tf_static (name.bag)

#play a rosbag 
 rosbag play (name.bag)

