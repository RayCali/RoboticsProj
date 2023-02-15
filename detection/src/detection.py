#!/usr/bin/python3

import numpy as np
import rospy
import ros_numpy as rnp
import torch
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from .....RoboticsProj_VisionModel import utils, detector

FOCAL_LENGTH = 1.93/1000 # focal lenth in m
BASELINE = 50/1000 # baseline in m

def imageCB(msg):
    # msg is of type Image, convert to torch tensor
    np_image = rnp.numpify(msg) # shape: (720,1280,3)
    torch_image = torch.from_numpy(np_image) # size: (720,1280,3)

    inference = detectionModel(torch_image) # size: (15,20,5)

    bbs = detectionModel.decode_output(inference, threshold=0.7)
    
    # only one image, so iterate over all bbs in that image
    # to extract their position and overlay them over the image
    for bb in bbs[0]:
        x = bb["x"]
        y = bb["y"]
        width = bb["width"]
        height = bb["height"]

        # extract center position of box
        center_x = x+int(width/2)
        center_y = y-int(height/2)

        # overlay box on image
        X = np.arange(x, x+width)
        Y = np.arange(y, y-height, step=-1)
        np_image[Y,X,0] = 255
        np_image[Y,X,1] = 0
        np_image[Y,X,2] = 0
    
    pubImg = rnp.msgify(Image,np_image)

    imgPub.publish(pubImg)
    


if __name__=="__main__":
    rospy.init_node("detection")

    imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB)
    posePub = rospy.Publisher("/detection/pose", PoseStamped, queue_size=10)
    imgPub = rospy.Publisher("detection/overlaid_bbs", Image, queue_size=10)

    # Load model
    detectionModel = utils.load_model(detector.Detector(),"~/RoboticsProj_VisionModel/models/det_2023-02-13_17-30-19-206874.pt", device="gpu")

    detectionModel.eval()