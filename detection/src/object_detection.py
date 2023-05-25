#!/usr/bin/python3

import numpy as np
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import ros_numpy as rnp
import torch
from torchvision import transforms
from torchvision.utils import draw_bounding_boxes, save_image
from torchvision.io import read_image
from scipy.ndimage import binary_erosion, binary_dilation
from sensor_msgs.msg import Image
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
from geometry_msgs.msg import PoseStamped, TransformStamped
import utils, detector
from msg_srv_pkg.msg import objectPoseStampedLst
from PIL import Image as pil
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from rospy import loginfo
from cv_bridge import CvBridge
import cv2
from shapely.geometry import Polygon
import os

FOCAL_LENGTH = 605.9197387695312
DEVICE = "cuda"
count = 0



def get_object_position(depthImg, center_x, center_y, img_center_x, img_center_y):
    # depthImg is of type Image, convert to torch tensor
    np_depthImg = rnp.numpify(depthImg) # shape: (480,640,1)
    # get depth value at center of bounding box
    center_x_offset = center_x-img_center_x
    center_y_offset = img_center_y-center_y
    depth = np_depthImg[center_y,center_x]
    # calculate distance to object
    z = depth*0.001 # depth in m
    # calculate x and y position of object
    x = z * (center_x_offset) / FOCAL_LENGTH
    y = z * (center_y_offset) / FOCAL_LENGTH

    object_position = np.array([x,y,z])
    # print(object_position)
    return object_position

def get_map_pose(position,msg_frame,msg_stamp):
    pose = PoseStamped()
    pose.header.frame_id = msg_frame
    pose.header.stamp = msg_stamp #rospy.Time.now()
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    
    # object_poses.PoseStamped.append(pose)
    # object_poses.object_class.append(label)
    # classes.append(label)
    try:
        #transform = tf_buffer.lookup_transform("map", msg_frame, msg_stamp, rospy.Duration(2))
        transform = tf_buffer.lookup_transform("map", msg_frame, rospy.Time(0), rospy.Duration(2))
        map_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        map_pose.pose.position.z = 0.0
        # fix orientation facing the robot
        try:
            #map_to_base = tf_buffer.lookup_transform("map", "base_link", msg_stamp, timeout=rospy.Duration(2))
            map_to_base = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2))
            rotation = np.array((map_to_base.transform.rotation.x, map_to_base.transform.rotation.y, map_to_base.transform.rotation.z, map_to_base.transform.rotation.w))
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(rotation)
            yaw = yaw + math.pi
            quat = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
            map_pose.pose.orientation.x = quat[0]
            map_pose.pose.orientation.y = quat[1]
            map_pose.pose.orientation.z = quat[2]
            map_pose.pose.orientation.w = quat[3]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            rospy.loginfo("Couldn't fix orientation of object facing the robot, using (0,0,0,1) orientation wrt map.")
            pose.pose.orientation.w = 1
        return map_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)
        return e
    
def get_baseLink_pose(position,msg_frame,msg_stamp):
    pose = PoseStamped()
    pose.header.frame_id = msg_frame
    pose.header.stamp = msg_stamp #rospy.Time.now()
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    
    try:
        #transform = tf_buffer.lookup_transform("base_link", msg_frame, msg_stamp, rospy.Duration(2))
        transform = tf_buffer.lookup_transform("base_link", msg_frame, rospy.Time(0), rospy.Duration(2))
        baseLink_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        baseLink_pose.pose.position.z = 0.0
        # fix orientation facing the robot
        try:
            #map_to_base = tf_buffer.lookup_transform("map", "base_link", msg_stamp, timeout=rospy.Duration(2))
            map_to_base = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2))
            rotation = np.array((map_to_base.transform.rotation.x, map_to_base.transform.rotation.y, map_to_base.transform.rotation.z, map_to_base.transform.rotation.w))
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(rotation)
            yaw = yaw + math.pi
            quat = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
            baseLink_pose.pose.orientation.x = quat[0]
            baseLink_pose.pose.orientation.y = quat[1]
            baseLink_pose.pose.orientation.z = quat[2]
            baseLink_pose.pose.orientation.w = quat[3]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            rospy.loginfo("Couldn't fix orientation of object facing the robot, using (0,0,0,1) orientation wrt map.")
            pose.pose.orientation.w = 1
        return baseLink_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)
        return e


def imageCB(msg: Image):
    global count
    image_frame_id  = msg.header.frame_id
    image_stamp = msg.header.stamp
    depthImg_rcvd = False
    try:
        depthImg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=0.1)
        depthImg_rcvd = True
    except rospy.ROSInterruptException:
        rospy.loginfo("No depth image received")
        return
    #imgPub.publish(msg)
    # msg is of type Image, convert to torch tensor
    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    np_image = rnp.numpify(msg) # shape: (480, 640, 3)
    
    image = transforms.ToTensor()(cv_image) # shape: (3,720,1280)
    image = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )(image)

    image = image.unsqueeze(0)
    image = image.to(DEVICE)
        
    with torch.no_grad():
        inference = detectionModel(image).cpu() # size: (15,20,5)
    
    bbs = detectionModel.decode_output(inference, threshold=0.7)[0]
    image = torch.from_numpy(np_image).permute(2,0,1)
    img_center_x = int(image.shape[2]/2)
    img_center_y = int(image.shape[1]/2)
    boxes = []
    labels = []
    scores = []
    positions = []
    for bbx in bbs:
        x, y, width, height, score, label = bbx["x"], bbx["y"], bbx["width"], bbx["height"], bbx["score"], bbx["category"]
        # extract center position of box
        centerbbx_x = int(x+int(width/2))
        centerbbx_y = int(y+int(height/2))    
    
        if depthImg_rcvd:
            object_position = get_object_position(depthImg, centerbbx_x, centerbbx_y, img_center_x, img_center_y)

            distance_to_object = object_position[2]
            if distance_to_object > 1.5 or distance_to_object < 0.2:
                # We have decided that this is a phantom detection and we should ignore it
                continue
        box = torch.tensor([x,y,x+width,y+height], dtype=torch.int).unsqueeze(0)
            
        for i in range(len(boxes)):
            if np.linalg.norm(positions[i] - object_position) < 0.05:
                if scores[i] >= score:
                    break
                else:
                    boxes[i] = box
                    labels[i] = label
                    scores[i] = score
                    if depthImg_rcvd:
                        positions[i] = object_position
                    break
            if i == len(boxes)-1 and score > 0.8:
                boxes.append(box)
                labels.append(label)
                scores.append(score)
                if depthImg_rcvd:
                    positions.append(object_position)
        
        if len(boxes) == 0 and score > 0.8:
            boxes.append(box)
            labels.append(label)
            scores.append(score)
            if depthImg_rcvd:
                positions.append(object_position)

    if len(boxes) > 0:
        boxes = torch.cat(boxes)
        # draw bounding boxes
        image = draw_bounding_boxes(image, boxes, width=5,
                          colors="green",labels=labels,
                          fill=False,font="/home/robot/Downloads/16020_FUTURAM.ttf",font_size=50)
        
    i = 1
    # transform and publish poses
    object_poses = objectPoseStampedLst()
    object_poses_baseLink = objectPoseStampedLst()
    if len(labels) > 0:
        count += 1
    s = str(len(labels)) + " new detections" + "\t " + str(count) + " frames with objects detected"
    rospy.loginfo(s)
    rospy.loginfo(labels)
    for pos, label in zip(positions,labels):
        try:
            map_pose = get_map_pose(pos,image_frame_id,image_stamp)
            object_poses.PoseStamped.append(map_pose)
            object_poses.object_class.append(label)

            # publish transform
            t = TransformStamped()
            t.header.stamp = image_stamp
            t.header.frame_id = "map"
            t.child_frame_id = "detection_"+label+str(i)
            
            
            t.transform.translation = map_pose.pose.position
            t.transform.rotation = map_pose.pose.orientation
            
            tfbroadcaster.sendTransform(t)
            i+=1
        except:
            loginfo("Failed to get map pose")
            continue

        try:
            baseLink_pose = get_baseLink_pose(pos,image_frame_id,image_stamp)
            object_poses_baseLink.PoseStamped.append(baseLink_pose)
            rospy.loginfo("Detected:" + label)
            object_poses_baseLink.object_class.append(label)
            #rospy.loginfo(baseLink_pose)

        except:
            loginfo("Failed to get baseLink pose")
            continue
        

    
    posesPub.publish(object_poses)

    baseLink_posePub.publish(object_poses_baseLink)
    
    pubImg = rnp.msgify(Image,image.permute(1,2,0).numpy(),encoding='rgb8')
    
    imgPub.publish(pubImg)


def proofCB(msg: Image):
    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    image = transforms.ToTensor()(cv_image) # shape: (3,720,1280)
    image = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )(image)
    image = image.unsqueeze(0)
    image = image.to(DEVICE)

    with torch.no_grad():
        inference = detectionModel(image).cpu() # size: (15,20,5)
    bbs = detectionModel.decode_output(inference, threshold=0.7)[0]

    max_bbx = None
    max_bbx_label = None
    max_score = 0
    for bbx in bbs:
        x, y, width, height, score, label = bbx["x"], bbx["y"], bbx["width"], bbx["height"], bbx["score"], bbx["category"]
        if score > max_score:
            max_score = score
            box = torch.tensor([x,y,x+width,y+height], dtype=torch.int).unsqueeze(0)
            max_bbx = box
            max_bbx_label = label

    boxes = torch.cat([max_bbx])
    np_image = rnp.numpify(msg) # shape: (480, 640, 3)
    image = torch.from_numpy(np_image).permute(2,0,1)
    image = draw_bounding_boxes(image, boxes, width=5,
                          colors="green",labels=[max_bbx_label],
                          fill=False,font="/home/robot/Downloads/16020_FUTURAM.ttf",font_size=50)
    
    files = [file for file in os.listdir('/home/robot/dd2419_ws/proof_images') if file.endswith('.png')]
    same_label_files = [file for file in files if file.startswith("ID:"+str(max_bbx_label))]
    name = '/home/robot/dd2419_ws/proof_images/' +"ID:"+ str(max_bbx_label) + "_" + str(len(same_label_files)+1) + ".png"
    toPIL = transforms.ToPILImage()
    image_pil = toPIL(image)
    image_pil.save(name)
    # save_image(image, name)

    return



if __name__=="__main__":
    
    rospy.init_node("object_detection")
    
    detectionModel = utils.load_model(detector.Detector(),"/home/robot/models/working_model/index.pt", device="cuda")
    detectionModel.eval()
    detectionModel = detectionModel.to(DEVICE)

    bridge = CvBridge()

    imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB,queue_size=10, buff_size=2**24)
    proofSub = rospy.Subscriber("proof", Image, proofCB, queue_size=1,  buff_size=2**24)

    posesPub = rospy.Publisher("/detection/pose", objectPoseStampedLst, queue_size=10)
    imgPub = rospy.Publisher("detection/overlaid_bbs", Image, queue_size=10)
    baseLink_posePub = rospy.Publisher("/detection/pose_baseLink", objectPoseStampedLst, queue_size=10)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()

    rospy.spin()