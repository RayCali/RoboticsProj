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
from msg_srv_pkg.srv import PickPose, PickPoseResponse, PickPoseRequest
from utils import *



class ArmCam:
    def __init__(self) -> None:
        rospy.loginfo("Initiating ArmCam instance...")
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.blobPub = rospy.Publisher("/blob", Image, queue_size=1)

        rospy.Service("/getPickPose", PickPose, self.getPickPose)


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

        self.Z = 0.221

        self.distortion_factors = [-0.50881066,  0.39447751, -0.00259297, -0.00138649, -0.23784509, 0.0, 0.0, 0.0]
        self.camera_matrix = [[517.03632655,   0.0, 312.03052029],
                              [0.0, 516.70216219, 252.01727667],
                              [0.0, 0.0, 1.0]]

        

    def getPickPose(self, req: PickPoseRequest):
        image = rospy.wait_for_message("/usb_cam/image_raw", Image)

        cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
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

            # Get pose in camera frame
            principal_x = self.camera_matrix[0][2]
            principal_y = self.camera_matrix[1][2]
            focal_x = self.camera_matrix[0][0]
            focal_y = self.camera_matrix[1][1]
            X = self.Z * (cX - principal_x) / focal_x
            Y = self.Z * (cY - principal_y) / focal_y

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "arm_cam"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = self.Z - 0.015
            pose_stamped.pose.position.y = -X
            pose_stamped.pose.position.z = -Y
            pose_stamped.pose.orientation.x = 0
            pose_stamped.pose.orientation.y = 0
            pose_stamped.pose.orientation.z = 0
            pose_stamped.pose.orientation.w = 1

            print("THETA:  ",theta*180/math.pi)

            resp = PickPoseResponse()
            resp.success = True
            resp.pose_stamped = pose_stamped
            resp.theta = theta

            return resp


        else:
            rospy.loginfo("No contours found")
            resp = PickPoseResponse()
            resp.success = False
            return resp

            

if __name__ == "__main__":
    rospy.init_node("arm_camera")
    rospy.loginfo("Arm Cam Node Started")
    try:
        armcam = ArmCam()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
            