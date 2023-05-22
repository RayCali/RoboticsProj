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
        self.proofPub = rospy.Publisher("/proof", Image, queue_size=1)

        rospy.Service("/srv/getPickPose/arm_camera/pickup", PickPose, self.getPickPose)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ball_radius = 0.045/2

        self.Z = 0.182# 0.221

        self.distortion_factors = np.array([-0.50881066,  0.39447751, -0.00259297, -0.00138649, -0.23784509, 0.0, 0.0, 0.0])
        self.camera_matrix = np.array([[517.03632655,   0.0, 312.03052029],
                                        [0.0, 516.70216219, 252.01727667],
                                        [0.0, 0.0, 1.0]])

        

    def getPickPose(self, req: PickPoseRequest):
        image = rospy.wait_for_message("/usb_cam/image_raw", Image)
        self.proofPub.publish(image)

        cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        h,  w = cv_image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_factors, (w,h), 1, (w,h))

        # undistort
        dst = cv2.undistort(cv_image, self.camera_matrix, self.distortion_factors, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]
        dst[0:y, :] = dst[y+h:-1, :] = dst[:, 0:x] = dst[:, x+w:-1] = 0
        gray = cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
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
        blob[y+h-27:y+h,x:x+w] = blob[y,x:x+w] = blob[y:y+h,x] = blob[y:y+h,x+h-1] = 255

        # Get contours
        cnts = cv2.findContours(blob, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts = list(cnts)
        for j, cnt in enumerate(cnts):
            if cv2.contourArea(cnt) < 200 or cv2.contourArea(cnt) > 15000:
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
            print("DIFF: ",abs(d1-d2))
            diff = abs(d1-d2)
            if diff < 10 or diff > 20:
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
            else:
                theta = math.pi/2

            # draw contour
            result = dst.copy()
            cv2.drawContours(result, [box], -1, (255,0,0), 2)
            cv2.circle(result, (cX, cY), 2, (0, 255, 0), -1)
            # principal_x = newcameramtx[0,2]
            # principal_y = newcameramtx[1,2]
            # cv2.circle(result, (int(principal_x), int(principal_y)), 10, (0, 255, 0), -1)
            
            im_with_keypoints_ROS = self.bridge.cv2_to_imgmsg(result, "rgb8")

            self.blobPub.publish(im_with_keypoints_ROS)

            # Get pose in camera frame
            principal_x = newcameramtx[0,2]
            principal_y = newcameramtx[1,2]
            focal_x = newcameramtx[0,0]
            focal_y = newcameramtx[1,1]
            X = self.Z * (cX - principal_x) / focal_x
            Y = self.Z * (cY - principal_y) / focal_y

            alpha_x = math.atan2(self.Z, X)
            alpha_y = math.atan2(self.Z, -Y)

            dx = self.ball_radius * math.cos(alpha_x)
            dy = self.ball_radius * math.cos(alpha_y)

            print("Alpha X: ",alpha_x*180/math.pi)
            print("Alpha Y: ",alpha_y*180/math.pi)
            print("X: ",X)
            print("Y: ",Y)
            print("DX: ",dx)
            print("DY: ",dy)

            # X = X + dx
            # Y = Y - dy

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "arm_cam"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = self.Z - 0.015
            pose_stamped.pose.position.y = -X
            pose_stamped.pose.position.z = -Y
            pose_stamped.pose.orientation.x = 0
            pose_stamped.pose.orientation.y = alpha_y
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
            