#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from math import sin, cos
import math
import cv2
from cv_bridge import CvBridge
from msg_srv_pkg.srv import Request, RequestResponse, RequestRequest
from utils import *


camera_position = [0.0, -0.24713861786666663, -1.0011208418666664, -1.801179757333333, 0.0, -1.7802358066666664]

class ArmCam:
    def __init__(self) -> None:
        rospy.loginfo("Initiating ArmCam instance...")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.blobPub = rospy.Publisher("/blob", Image, queue_size=1)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 300
        params.filterByArea = True
        params.minArea = 200
        params.filterByCircularity = False

        self.blob_detector = cv2.SimpleBlobDetector_create(params)
        
        self.once = True

    def image_callback(self, msg):
        rospy.loginfo("Huuuuuuu")
        if self.once:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            blobs_dog = cv2.GaussianBlur(cv_image, (3,3), 0) - cv2.GaussianBlur(cv_image, (15,15), 0)
    
            cv_im_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            thresh = cv2.adaptiveThreshold(cv_im_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 201, 1)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
            blob = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
            blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, kernel)
            
            keypoints = self.blob_detector.detect(blobs_dog)
            im_with_keypoints = cv2.drawKeypoints(cv_im_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            im_with_keypoints_ROS = self.bridge.cv2_to_imgmsg(blob)

            self.blobPub.publish(im_with_keypoints_ROS)

            

if __name__ == "__main__":
    rospy.init_node("arm_camera")
    rospy.loginfo("Arm Cam Node Started")
    try:
        armcam = ArmCam()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
            