# NOTES
#### Staffans Junk
# token for pushing and pulling
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
