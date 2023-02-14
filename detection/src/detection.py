#!/usr/bin/python3

import numpy as np
import rospy
import ros_numpy as rnp
import torch
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from .....RoboticsProj_VisionModel import utils, detector


def imageCB(msg):
    # msg is of type Image, convert to torch tensor
    np_image = rnp.numpify(msg) # shape: (720,1280,3)
    torch_image = torch.from_numpy(np_image) # size: (720,1280,3)

    inference = detectionModel(torch_image) # size: (15,20,5)

    bbs = detectionModel.decode_output(inference, threshold=0.7)

    
    utils.add_bounding_boxes()
    


if __name__=="__main__":
    rospy.init_node("detection")

    imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB)
    posePub = rospy.Publisher("/detection/pose", PoseStamped, queue_size=10)

    # Load model
    detectionModel = utils.load_model(detector.Detector(),"~/RoboticsProj_VisionModel/models/det_2023-02-13_17-20-07-496421.pt", device="gpu")

    detectionModel.eval()