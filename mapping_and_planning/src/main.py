#!/usr/bin/python3

import numpy as np
import math
import rospy
from rospy import loginfo
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from geometry_msgs.msg import PoseStamped
# import torch
# from torchvision import transforms
from msg_srv_pkg.msg import objectPoseStampedLst
from gridmapping import Map
from global_explorer import getMostValuedCell
from path_planner import doPlan


if __name__ == "__main__":

    rospy.init_node("mapping_and_planning")
    pose_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)
    m = Map(True, 11, 11, 0.05)
    rospy.sleep(1)
    m.doPublish()
    doPlan(m)
    x_ind_goal, y_ind_goal = getMostValuedCell(m.matrix, m.grid.info.width, m.grid.info.height)
    x = m.grid.info.resolution * x_ind_goal
    y = m.grid.info.resolution * y_ind_goal
    ps = PoseStamped()
    ps.pose.position.x = x
    ps.pose.position.y = y
    pose_pub.publish(ps)
    # print(getMostValuedCell(m.matrix, m.grid.info.width, m.grid.info.height))
    while True:
        rospy.sleep(0.05)
        m.doPublish()
        
