#!/usr/bin/python3

import numpy as np
import math
import rospy
from rospy import loginfo
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
# import torch
# from torchvision import transforms
from detection.msg import objectPoseStampedLst
from gridmapping import Map
from global_explorer import getValues, getMostValuedCell


if __name__ == "__main__":

    rospy.init_node("mapping_and_planning")
    m = Map(True, 11, 11)
    rospy.sleep(1)
    m.doPublish()
    while True:
        rospy.sleep(1)
        m.doPublish()
        getMostValuedCell()
