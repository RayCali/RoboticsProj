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
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.BORDER_REPLICATE, cv2.THRESH_BINARY, 69, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13,13))
        blob = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        blob = cv2.morphologyEx(blob, cv2.MORPH_DILATE, kernel)

        
        blob = 255-blob
        h, w = blob.shape
        blob[0,:] = blob[:,0] = blob[h-150:h-1,:] = blob[:,w-1] = 255

        # Get contours
        cnts = cv2.findContours(blob, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = list(cnts)
        for j, cnt in enumerate(cnts):
            if cv2.contourArea(cnt) < 3500 or cv2.contourArea(cnt) > 30000:
                cnts[j] = "remove"
        # filtered_cnts = list(filter(lambda a: a != 2, cnts))
        filtered_cnts = [cnt for cnt in cnts if cnt != "remove"]

        if len(filtered_cnts) > 0:
            biggest_contour = max(filtered_cnts, key=cv2.contourArea)

            rect = cv2.minAreaRect(biggest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Get centroid
            M = cv2.moments(box)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            d1 = math.sqrt((box[0][0] - box [1][0])**2 + (box[0][1] - box [1][1])**2)
            d2 = math.sqrt((box[1][0] - box [2][0])**2 + (box[1][1] - box [2][1])**2)
            
            if d1 > d2:
                theta = -(math.atan2(box[0][1] - box[1][1], box[0][0] - box[1][0]) - math.pi/2)
                theta = theta % math.pi
                if theta > math.pi/2:
                    theta = theta - math.pi
                if theta < -math.pi/2:
                    theta = theta + math.pi
            else:
                theta = -(math.atan2(box[1][1] - box[2][1], box[1][0] - box[2][0]) - math.pi/2)
                theta = theta % math.pi
                if theta > math.pi/2:
                    theta = theta - math.pi
                if theta < -math.pi/2:
                    theta = theta + math.pi

            # draw contour
            result = cv_image.copy()
            cv2.drawContours(result, [box], -1, (0,0,255), 3)
            cv2.circle(result, (cX, cY), 7, (0, 255, 0), -1)
            
            im_with_keypoints_ROS = self.bridge.cv2_to_imgmsg(result, "bgr8")

            self.blobPub.publish(im_with_keypoints_ROS)

            

if __name__ == "__main__":
    rospy.init_node("arm_camera")
    rospy.loginfo("Arm Cam Node Started")
    try:
        armcam = ArmCam()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
            