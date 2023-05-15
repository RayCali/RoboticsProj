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
    print("Hello world!")
    images = glob.glob('/home/robot/test_images_usbcam/*.jpg')
    print("I got {} images".format(len(images)))
    distortion_factors = np.array([-0.50881066,  0.39447751, -0.00259297, -0.00138649, -0.23784509, 0.0, 0.0, 0.0])
    camera_matrix = np.array([[517.03632655,   0.0, 312.03052029],
                              [0.0, 516.70216219, 252.01727667],
                              [0.0, 0.0, 1.0]])
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
        h,  w = cv_image.shape[:2]
        # camera_matrix = np.array(camera_matrix)
        # distortion_factors = np.array(distortion_factors)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_factors, (w,h), 1, (w,h))
        print("newcameramtx: {}".format(newcameramtx[0,2]))

        # undistort
        dst = cv2.undistort(cv_image, camera_matrix, distortion_factors, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        # dst[y:y+h, x:x+w]
        dst[0:y, :] = dst[y+h:-1, :] = dst[:, 0:x] = dst[:, x+w:-1] = 0

        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
        # blurred = cv2.GaussianBlur(dst, (5,5), 0.5)
        # gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.BORDER_REPLICATE, cv2.THRESH_BINARY, 11, 4)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        blob0 = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        blob1 = cv2.morphologyEx(blob0, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        blob2 = cv2.morphologyEx(blob1, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        blob3 = cv2.morphologyEx(blob2, cv2.MORPH_CLOSE, kernel)
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # blob = cv2.morphologyEx(blob, cv2.MORPH_DILATE, kernel)

        
        blob = 255-blob3
        h, w = blob.shape
        blob[0,:] = blob[:,0] = blob[h-20:h-1,:] = blob[:,w-1] = 255
        # cv2.imshow('blob', thresh)
        # cv2.waitKey()
        # cv2.imshow('blob', blob0)
        # cv2.waitKey()
        # cv2.imshow('blob', blob1)
        # cv2.waitKey()
        # cv2.imshow('blob', blob2)
        # cv2.waitKey()
        # cv2.imshow('blob', blob3)
        # cv2.waitKey()
        # cv2.imshow('blob', blob)
        # cv2.waitKey()
        print(blob.shape)

        # Get contours
        cnts = cv2.findContours(blob, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        result = dst.copy()
        cv2.drawContours(result, cnts, -1, (0,0,255), 3)
        cnts = list(cnts)
        remove_idxs = []
        for j, cnt in enumerate(cnts):
            print("Contour {} has area {}".format(j, cv2.contourArea(cnt)))
            if cv2.contourArea(cnt) < 200 or cv2.contourArea(cnt) > 15000:
                cnts[j] = "remove"
        # print(cnts)
        # filtered_cnts = list(filter(lambda a: a != 2, cnts))
        filtered_cnts = [cnt for cnt in cnts if cnt != "remove"]
        # big_contour = max(filtered_cnts, key=cv2.contourArea)

        # test blob size
        # blob_area_thresh = 1000
        # # blob_area = cv2.contourArea(big_contour)
        # if blob_area < blob_area_thresh:
        #     print("Blob Is Too Small")

        # draw contour
        result = dst.copy()
        cv2.drawContours(result, filtered_cnts, -1, (0,0,255), 3)


        cv2.imshow('blob', result)
        cv2.waitKey()
