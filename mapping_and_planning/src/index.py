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
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import matplotlib.pyplot as plt
from PIL import Image as pil
from gridmapping import Mapper
import yaml
from detection.msg import objectPoseStamped
from gridmapping_new import Map

# This is just so that it is easier to read the code
def doOneUpdate():
    frames_dict = yaml.safe_load(tfBuffer.all_frames_as_yaml())
    for key in frames_dict.keys():
        if not m.keyLogged(key):
            t = tflistener.getLatestCommonTime(frames_dict[key], "/map")
            ts: TransformStamped = tfBuffer.lookupTransform(
                frames_dict[key], "/map", t)
            m.doInterpreteMSG(msg=ts, id="UNK")

def doUpdate(msg: objectPoseStamped):
    loginfo(msg)


if __name__ == "__main__":
    rospy.init_node("mapping_and_planning")
    m = Map()
    #m.doAnimate()

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    st = tf2_ros.StaticTransformBroadcaster()
    map = Map()
    rospy.spin()