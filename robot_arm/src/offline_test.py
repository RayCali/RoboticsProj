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
from utils import *
import glob



if __name__ == "__main__":
    images = glob.glob('/home/grumpy/usb_images/*.jpg')
    # params = cv2.SimpleBlobDetector_Params()

    # # Change thresholds
    # params.minThreshold = 10
    # params.maxThreshold = 300
    # params.filterByArea = True
    # params.minArea = 200
    # params.filterByCircularity = False

    # blob_detector = cv2.SimpleBlobDetector_create(params)

    for i, img in enumerate(images):
        cv_image = cv2.imread(img)
        blurred = cv2.GaussianBlur(cv_image, (7,7), 10)
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
        print(blob.shape)

        # Get contours
        cnts = cv2.findContours(blob, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = list(cnts)
        print("------------------------------")
        print("ITERATIONNNNNN:",i)
        print(len(cnts))
        remove_idxs = []
        for j, cnt in enumerate(cnts):
            print("------------------")
            print(cv2.contourArea(cnt))
            if cv2.contourArea(cnt) < 3500 or cv2.contourArea(cnt) > 30000:
                print("Remove: ",cv2.contourArea(cnt))
                cnts[j] = "remove"
            print("------------------")
        print(j)
        # print(cnts)
        print(remove_idxs)
        # filtered_cnts = list(filter(lambda a: a != 2, cnts))
        filtered_cnts = [cnt for cnt in cnts if cnt != "remove"]
        # big_contour = max(filtered_cnts, key=cv2.contourArea)

        # test blob size
        # blob_area_thresh = 1000
        # # blob_area = cv2.contourArea(big_contour)
        # if blob_area < blob_area_thresh:
        #     print("Blob Is Too Small")

        # draw contour
        result = cv_image.copy()
        cv2.drawContours(result, filtered_cnts, -1, (0,0,255), 3)


        cv2.imshow('blob', result)
        cv2.waitKey(2000)
