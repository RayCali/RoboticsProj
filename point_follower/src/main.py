#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
from robp_msgs.msg import Encoders
from aruco_msgs.msg import MarkerArray
from robp_msgs.msg import DutyCycles
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from detection.msg import objectPoseStampedLst
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import tf
from point_follower.srv import Node, Moveto, MovetoResponse, MovetoRequest
SUCCESS = 1
FAILURE = -1
RUNNING = 0


class path(object):
    def __init__(self):
        # ROS Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ROS Subscribers
        # self.goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.tracker, queue_size=1) # has to be the pose of the postion we want to go to
        
        self.done_once = False
        self.rate = rospy.Rate(20)
        s = rospy.Service('/moveto', Moveto, self.tracker)

        
    def tracker(self,req:MovetoRequest):
        if not self.done_once:
            self.cam_pose = PoseStamped()
            self.twist = Twist()
            rospy.sleep(2)
            # if len(msg.PoseStamped) == 0:
            #     rospy.loginfo("No object detected")
            #     exit()
            
            self.cam_pose = req.goal
            # self.rate = rospy.Rate(20)
            # rospy.Time(0) = msg.header.stamp
            # rospy.Time(0) = msg.header.stamp
            # self.cam_pose.header.frame_id = msg.header.frame_id
            # self.cam_pose.pose = msg.pose
            try:
                self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
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
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y

            while math.atan2(self.inc_y, self.inc_x) > 0.05: # or math.atan2(inc_y, inc_x) < -0.2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.7
                rospy.loginfo("Turning left")
                rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
            
            self.twist.angular.z = 0.0
            self.pub_twist.publish(self.twist)
            self.rate.sleep()
            rospy.sleep(1)
            self.acceleration = 0.01
            self.deceleration = 0.08
            try:
                self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            self.inc_x = self.goal_pose.pose.position.x
            self.inc_y = self.goal_pose.pose.position.y

            while math.sqrt(self.inc_x**2 + self.inc_y**2) > 0.05:
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y

                if self.twist.linear.x < 0.6 and math.sqrt(self.inc_x**2 + self.inc_y**2)>self.twist.linear.x**2/(2*self.deceleration):
                    self.twist.linear.x += self.acceleration
                    rospy.loginfo(self.twist.linear.x)

                elif self.twist.linear.x >= 0.6 and math.sqrt(self.inc_x**2 + self.inc_y**2)>self.twist.linear.x**2/(2*self.deceleration): #eller acceleration
                    self.twist.linear.x = 0.6

                else:
                    self.twist.linear.x -= self.deceleration
                    rospy.loginfo("Decelerating")
                    rospy.loginfo(self.twist.linear.x)

                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x= self.goal_pose.pose.position.x
                self.inc_y= self.goal_pose.pose.position.y
                if math.atan2(self.inc_y, self.inc_x) > 0.05:
                    self.twist.angular.z = 0.2 #either -0.2 or 0.2
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()

                if math.atan2(self.inc_y, self.inc_x) < -0.05:
                    self.twist.angular.z = -0.2 #either -0.2 or 0.2
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()
            
            while math.atan2(self.inc_y, self.inc_x)< -0.02: # or math.atan2(inc_y, inc_x) < -0.2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.7
                rospy.loginfo("Turning right")
                rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y

            while math.atan2(self.inc_y, self.inc_x) > 0.02: # or math.atan2(inc_y, inc_x) < -0.2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.7
                rospy.loginfo("Turning left")
                rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
            self.done_once = True
        rospy.loginfo('You have reached the goal')
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub_twist.publish(self.twist)
        return MovetoResponse(SUCCESS)
        # self.rate.sleep()
        # try:
        #     self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
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
        #         self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
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
    rospy.loginfo("Starting path_tracker node")
    getCanIGetThereWithoutAnyCollisions = rospy.ServiceProxy('get_can_i_get_there_without_any_collisions', Node)
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
