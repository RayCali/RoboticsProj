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
from msg_srv_pkg.srv import Request, RequestRequest, RequestResponse
from global_explorer import getMostValuedCell
from typing import List
from config import SUCCESS, RUNNING, FAILURE


if __name__ == "__main__":

    rospy.init_node("path_client")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) 
    listener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2))
    #x = 1
    #y = 0.5
    x = 2.0
    y = 1



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


    goal_pub = rospy.Publisher("/mostValuedCell", PoseStamped, queue_size=10)


    ps: PoseStamped = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    
    ps.header.frame_id = "map"
    # x,y position of the anchor
    #x = transform.transform.translation.x
    #y = transform.transform.translation.y
    ps.pose.position.x = x
    ps.pose.position.y = y

    rospy.sleep(1)
    tfbroadcaster.sendTransform(t)
    goal_pub.publish(ps)

    rospy.sleep(2)

    
   
    path_planner = rospy.ServiceProxy("/srv/doPlanpath/mapping_and_planning/brain", Request)
    resp = path_planner(RequestRequest())



