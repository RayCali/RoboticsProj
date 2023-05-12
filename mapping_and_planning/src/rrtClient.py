#!/usr/bin/python3
import os

import numpy as np
import math
import rospy
from rospy import loginfo
import tf2_ros
import tf_conversions
# import torch
# from torchvision import transforms
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Quaternion, Vector3
from msg_srv_pkg.srv import Request, RequestRequest, RequestResponse
from global_explorer import getMostValuedCell
import tf2_geometry_msgs

from typing import List
from config import SUCCESS, RUNNING, FAILURE
table =  {
            -1: "FAILURE",
            0: "RUNNING",
            1: "SUCCESS"
        }

if __name__ == "__main__":

    rospy.init_node("path_client")
    os.system('cls' if os.name == 'nt' else 'clear')

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) 
    listener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    transform = tf_buffer.lookup_transform("map", "arucomap", rospy.Time(0), rospy.Duration(2))
    stamp = rospy.Time.now()

    pose_in_arucomapframe = PoseStamped()
    pose_in_arucomapframe.header.stamp = stamp
    pose_in_arucomapframe.header.frame_id = "arucomap"
    pose_in_arucomapframe.pose.position.x = -3
    pose_in_arucomapframe.pose.position.y = -1
    pose_in_arucomapframe.pose.orientation.w = 1

    pose_in_mapframe = tf2_geometry_msgs.do_transform_pose(pose_in_arucomapframe, transform)


    x = 1
    y = 0.5
    # x = 0 + transform.transform.translation.x
    # y = 1 + transform.transform.translation.y



    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "goal"
    t.transform.translation.x = pose_in_mapframe.pose.position.x
    t.transform.translation.y = pose_in_mapframe.pose.position.y
    t.transform.rotation.w = 1


    goal_pub = rospy.Publisher("/mostValuedCell", PoseStamped, queue_size=10)


    rospy.sleep(1)
    tfbroadcaster.sendTransform(t)
    goal_pub.publish(pose_in_mapframe)

    rospy.sleep(2)

    
   
    path_planner = rospy.ServiceProxy("/srv/doPlanpath/mapping_and_planning/brain", Request)
    res = path_planner(RequestRequest())
    rospy.loginfo("Result: "+ table[res.success])
        

