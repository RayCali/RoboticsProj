#!/usr/bin/python3
import rospy
import numpy as np
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import tf_conversions
import tf2_geometry_msgs
import math
tf_buffer = None #tf buffer length
listener = None
br = None
st = None
ifseenanchor = False


N = 10
mask: np.array = (
    [[2 for i in range(N)] for i in range(N)]
)

def getValues(m: Map):
    return 