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


if __name__ == "__main__":

    
    m = Map(plot=True, height=1000, width=1000, resolution=0.01)
    print(m.map.data.shape)
    m.doAnimate()
<<<<<<< HEAD
    print(m.map.data)
    
=======
    tfBuffer = tf2_ros.Buffer(rospy.Duration(1.0))
    tflistener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        doOneUpdate()
>>>>>>> 8c26916838cea979a3b6191a3c8ef3a18e8fb4a0
