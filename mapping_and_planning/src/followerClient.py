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
from typing import List
from config import SUCCESS, RUNNING, FAILURE
from nav_msgs.msg import Path

table =  {
            -1: "FAILURE",
            0: "RUNNING",
            1: "SUCCESS"
        }


if __name__ == "__main__":

    rospy.init_node("path_client")
    os.system('cls' if os.name == 'nt' else 'clear')
    proxy = rospy.ServiceProxy("/srv/doMoveAlongPath/path_follower/brain", Request)
    while not rospy.is_shutdown():
        res = proxy(RequestRequest())
        rospy.loginfo("Result: "+ table[res.success])
        rospy.sleep(1)
        if res.success == SUCCESS or res.success == FAILURE:
            break
        
    rospy.loginfo("Done!")

