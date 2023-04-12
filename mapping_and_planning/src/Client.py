#!/usr/bin/python3

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
from msg_srv_pkg.srv import Moveto, MovetoResponse, Request, RequestResponse, MovetoRequest
from global_explorer import getMostValuedCell
from typing import List
from config import SUCCESS, RUNNING, FAILURE


if __name__ == "__main__":

    rospy.init_node("path_client")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) 
    listener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    transform = tf_buffer.lookup_transform("map", "arucomap", rospy.Time(0), rospy.Duration(2))
    x = transform.transform.translation.x + 1.20
    y = transform.transform.translation.y

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "goal"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1.0
    rospy.sleep(1)
    tfbroadcaster.sendTransform(t)
    
    

    path_planner = rospy.ServiceProxy("pathPlanner", Moveto)
    ps = PoseStamped()
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    ps.pose = pose

    resp = path_planner(ps)


