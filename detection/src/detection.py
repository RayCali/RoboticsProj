#!/usr/bin/python3

import numpy as np
import rospy
import torch
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped



def imageCB(msg):
    # msg is of type Image, convert to torch tensor

    # Load model and input image
    pass


if __name__=="__main__":
    rospy.init_node("detection")

    imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB)
    posePub = rospy.Publisher("/detection/pose", PoseStamped, queue_size=10)