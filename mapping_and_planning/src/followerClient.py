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
from nav_msgs.msg import Path


if __name__ == "__main__":

    rospy.init_node("path_client")
    proxy = rospy.ServiceProxy("/srv/doMoveAlongPath/path_follower/brain", Request)
    while not rospy.is_shutdown():
        res = proxy(RequestRequest())
        print(res)
        rospy.sleep(1)
        if res.success == SUCCESS:
            break
        
    print("Done!")

