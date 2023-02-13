#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
from robp_msgs.msg import Encoders
from aruco_msgs.msg import MarkerArray
from robp_msgs.msg import DutyCycles
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import tf


x = 0.0
y = 0.0
theta = 0.0
# marker_ID = 0
# marker_position = Point()
# marker_orientation = Quaternion()
# marker_pose = Pose()

class path(object):
    def __init__(self):
        self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.tracker)
        
        print('tracker hej')

    def tracker(self,msg:PoseStamped):
        global x, y, theta
        rate = rospy.Rate(20)

        self.cam_pose = PoseStamped()
        twist = Twist()
        

        while not rospy.is_shutdown():
            print('tracker hej')
            msgTime = msg.header.stamp
            print("Hej!")
            self.cam_pose.header = msg.header
            self.cam_pose.pose = msg.pose

            try:
                trans_base = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(6.0))
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, trans_base)
                print('jkdlsakldfjkdfalkfa')
                # return self.goal_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            
            print('mamke robin')

            # print(self.goal_pose)

            inc_x = self.goal_pose.pose.position.x - x
            inc_y = self.goal_pose.pose.position.y - y
            print(inc_x, inc_y)

            if math.sqrt(inc_x**2 + inc_y**2)<0.1:
                print('You have reached the goal')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub_twist.publish(twist)
                break

            else:
                angle_to_goal = math.atan2(inc_y, inc_x)
                twist.linear.x = 5.0
                twist.angular.z = angle_to_goal
                pub_twist.publish(twist)
                print('Moving towards the goal')
        



        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_tracker")

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    # tfListener = tf.TransformListener()


    # rospy.Subscriber("/aruco/markers", MarkerArray, display_markers)
    
    pub_duty = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    try:
        planner = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
