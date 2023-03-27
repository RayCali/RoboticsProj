#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
from robp_msgs.msg import Encoders
from aruco_msgs.msg import MarkerArray
from robp_msgs.msg import DutyCycles
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from detection.msg import objectPoseStampedLst
from typing import List
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import tf



class path(object):
    def __init__(self):
        # ROS Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ROS Subscribers
        self.goal = rospy.Subscriber("detection/pose", objectPoseStampedLst, self.tracker)
        
        self.done_once = False
        self.rate = rospy.Rate(20)

        
    def tracker(self,msg:PoseStamped):
        # if not self.done_once:
        self.cam_pose = PoseStamped()
        self.twist = Twist()
        

        # self.rate = rospy.Rate(20)
        self.msgTime = msg.header.stamp
        self.cam_pose.header.stamp = msg.header.stamp
        self.cam_pose.header.frame_id = msg.header.frame_id
        self.cam_pose.pose = msg.pose

        try:
            self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(0.5))
            self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform point from odom frame to base_link frame")
            pass
        
        self.rotation = self.goal_pose.pose.orientation.z
        self.inc_x = self.goal_pose.pose.position.x
        self.inc_y = self.goal_pose.pose.position.y

        # rospy.loginfo(abs(rotation))
        while math.atan2(self.inc_y, self.inc_x)< -0.05: # or math.atan2(inc_y, inc_x) < -0.2:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.7
            rospy.loginfo("Turning right")
            rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
            self.pub_twist.publish(self.twist)
            self.rate.sleep()
            try:
                self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            self.inc_x = self.goal_pose.pose.position.x
            self.inc_y = self.goal_pose.pose.position.y

    #     while math.atan2(self.inc_y, self.inc_x) > 0.05: # or math.atan2(inc_y, inc_x) < -0.2:
    #         self.twist.linear.x = 0.0
    #         self.twist.angular.z = 0.7
    #         rospy.loginfo("Turning left")
    #         rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
    #         self.pub_twist.publish(self.twist)
    #         self.rate.sleep()
    #         try:
    #             self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
    #             self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             rospy.logerr("Failed to transform point from odom frame to base_link frame")
    #             pass
    #         self.inc_x = self.goal_pose.pose.position.x
    #         self.inc_y = self.goal_pose.pose.position.y
        
    #     self.twist.angular.z = 0.0
    #     self.pub_twist.publish(self.twist)
    #     self.rate.sleep()
    #     rospy.sleep(1)
    #     self.acceleration = 0.01
    #     self.deceleration = 0.08
    #     try:
    #         self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
    #         self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         rospy.logerr("Failed to transform point from odom frame to base_link frame")
    #         pass
    #     self.inc_x = self.goal_pose.pose.position.x
    #     self.inc_y = self.goal_pose.pose.position.y

    #     while math.sqrt(self.inc_x**2 + self.inc_y**2) > 0.05:
    #         try:
    #             self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
    #             self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             rospy.logerr("Failed to transform point from odom frame to base_link frame")
    #             pass
    #         self.inc_x = self.goal_pose.pose.position.x
    #         self.inc_y = self.goal_pose.pose.position.y

    #         if self.twist.linear.x < 0.6 and math.sqrt(self.inc_x**2 + self.inc_y**2)>self.twist.linear.x**2/(2*self.deceleration):
    #             self.twist.linear.x += self.acceleration
    #             rospy.loginfo(self.twist.linear.x)

    #         elif self.twist.linear.x >= 0.6 and math.sqrt(self.inc_x**2 + self.inc_y**2)>self.twist.linear.x**2/(2*self.deceleration): #eller acceleration
    #             self.twist.linear.x = 0.6

    #         else:
    #             self.twist.linear.x -= self.deceleration
    #             rospy.loginfo("Decelerating")
    #             rospy.loginfo(self.twist.linear.x)

    #         self.pub_twist.publish(self.twist)
    #         self.rate.sleep()
    #         try:
    #             self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
    #             self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             rospy.logerr("Failed to transform point from odom frame to base_link frame")
    #             pass
    #         self.inc_x= self.goal_pose.pose.position.x
    #         self.inc_y= self.goal_pose.pose.position.y
    #         if math.atan2(self.inc_y, self.inc_x) > 0.08:
    #             self.twist.angular.z = 0.1 #either -0.2 or 0.2
    #             self.pub_twist.publish(self.twist)
    #             self.rate.sleep()

    #         if math.atan2(self.inc_y, self.inc_x) < -0.08:
    #             self.twist.angular.z = -0.1 #either -0.2 or 0.2
    #             self.pub_twist.publish(self.twist)
    #             self.rate.sleep()
    
    # rospy.loginfo('You have reached the goal')
    # self.twist.linear.x = 0.0
    # self.twist.angular.z = 0.0
    # self.pub_twist.publish(self.twist)
    # self.rate.sleep()
    # try:
    #     self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
    #     self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         rospy.logerr("Failed to transform point from odom frame to base_link frame")
    #         pass
    # self.inc_x= self.goal_pose.pose.position.x
    # self.inc_y= self.goal_pose.pose.position.y
    # if math.sqrt(self.inc_x**2 + self.inc_y**2) < 0.05:
    #     self.done_once = True
    #     self.twist.linear.x = 0.0
    #     self.twist.linear.z = 0.0
    #     self.pub_twist.publish(self.twist)
    #     self.rate.sleep()
    #     try:
    #         self.trans = tfBuffer.lookup_transform("base_link", "odom", self.msgTime, timeout=rospy.Duration(2.0))
    #         self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             rospy.logerr("Failed to transform point from odom frame to base_link frame")
    #             pass
    #     self.inc_x= self.goal_pose.pose.position.x
    #     self.inc_y= self.goal_pose.pose.position.y
    #     # rospy.sleep(1)
    # else:
    #     rospy.loginfo('Going towards next goal')
    #     self.done_once = False

if __name__ == "__main__":
    rospy.init_node("point_follower")
    rospy.loginfo("Starting point_follower node")

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(2)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()