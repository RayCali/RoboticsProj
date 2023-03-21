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


if __name__=="__main__":
    

    m = Mapper()
    # m.makeoccupancygrid()
    m.doAnimate()
