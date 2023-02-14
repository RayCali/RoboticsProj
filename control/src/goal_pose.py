#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose
from robp_msgs.msg import Encoders
from aruco_msgs.msg import MarkerArray
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped




def callback(msg:PoseStamped):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.position.x = msg.pose.position.x
    pose_msg.pose.position.y = msg.pose.position.y
    pose_msg.pose.position.z = msg.pose.position.z
    pose_msg.pose.orientation.x = msg.pose.orientation.x
    pose_msg.pose.orientation.y = msg.pose.orientation.y
    pose_msg.pose.orientation.z = msg.pose.orientation.z
    pose_msg.pose.orientation.w = msg.pose.orientation.w
    rospy.loginfo(pose_msg)
    pub.publish(pose_msg)

    while not rospy.is_shutdown():
        rospy.loginfo(pose_msg)
        pub.publish(pose_msg)
        rospy.sleep(1)

rospy.init_node('goal_pose')
pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=10)
sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped,callback)