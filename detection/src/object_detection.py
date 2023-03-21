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
from torchvision.utils import draw_bounding_boxes
from torchvision.io import read_image
from scipy.ndimage import binary_erosion, binary_dilation
from sensor_msgs.msg import Image, PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import utils, detector
from detection.msg import centerpointArray, boundingboxArray, boundingboxMsg
from PIL import Image as pil
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from rospy import loginfo
from cv_bridge import CvBridge
import cv2
from shapely.geometry import Polygon

FOCAL_LENGTH = 1.93/1000 # focal lenth in m
BASELINE = 50/1000 # baseline in m (distance between the two infrared cams)
BASELINE_1 = 65/1000 # distance in m between color cam and right infrared cam
BASELINE_2 = 15/1000 # distance in m between color cam and left infrared cam
CENTERIMG_X = 640 # center of image in x direction
CENTERIMG_Y = 360 # center of image in y direction
PLUSHIE_HEIGHT = 0.09 # height of plushie in m
PLUSHIE_WIDTH = 0.035 # width of plushie in m
PLUSHIE_DEPTH = 0.06 # depth of plushie in m
CUBE_SIDELENGTH = 0.03 # side length of cube in m
BALL_DIAMETER = 0.04 # diameter of ball in m
DEVICE = "cuda"

def get_object_position(depthImg, center_x, center_y):
    # depthImg is of type Image, convert to torch tensor
    np_depthImg = np_image = rnp.numpify(depthImg) # shape: (720,1280,1)
    # get depth value at center of bounding box
    # print(center_x)
    depth = np_depthImg[center_y,center_x]
    # calculate distance to object
    # dist = (FOCAL_LENGTH * BASELINE) / depth
    z = depth*0.001 # depth in m
    # calculate x and y position of object
    x = z * center_x / FOCAL_LENGTH
    y = z * center_y / FOCAL_LENGTH

    object_position = np.array([x,y,z])
    print(object_position)
    return object_position


def imageCB(msg: Image):
    depthImg_rcvd = False
    try:
        depthImg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=0.1)
        depthImg_rcvd = True
    except rospy.ROSInterruptException:
        rospy.loginfo("No depth image received")
    #imgPub.publish(msg)
    # msg is of type Image, convert to torch tensor
    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
    np_image = rnp.numpify(msg) # shape: (480, 640, 3)
    
    image = transforms.ToTensor()(cv_image) # shape: (3,720,1280)
    image = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )(image)
    image = image.to(DEVICE)
    
    image = image.unsqueeze(0)
    #loginfo(image)
    with torch.no_grad():
        inference = detectionModel(image).cpu() # size: (15,20,5)
    
    bbs = detectionModel.decode_output(inference, threshold=0.7)[0]
    image = torch.from_numpy(np_image).permute(2,0,1)
    image_width = image.shape[2]
    image_height = image.shape[1]
    errors = centerpointArray()
    boxes = []
    labels = []
    scores = []
    positions = []
    for bbx in bbs:
        # label=[]
        x, y, width, height, score, label = bbx["x"], bbx["y"], bbx["width"], bbx["height"], bbx["score"], bbx["category"]
        # extract center position of box
        centerbbx_x = int(x+int(width/2))
        centerbbx_y = int(y+int(height/2))
        error_x = centerbbx_x - image_width/2
        v3s = Vector3Stamped()
        v3s.vector.x = error_x
        errors.objects_in_view.append(v3s)
        
        if depthImg_rcvd:
            object_position = get_object_position(depthImg, centerbbx_x, centerbbx_y)

        # deviation of bounding box from center (only in x)
        # overlay box on image
        #X = np.arange(x, x+width)
        #Y = np.arange(y, y-height, step=-1)
        box = torch.tensor([x,y,x+width,y+height], dtype=torch.int).unsqueeze(0)
        corners = [[int(box[0,0]),int(box[0,1])], [int(box[0,2]),int(box[0,1])],
                    [int(box[0,2]),int(box[0,3])], [int(box[0,0]),int(box[0,3])]]
        # print(corners)
        polygon = Polygon(corners)
        # print(box)
        # box_float = torch.tensor(box, dtype=torch.float)
        # print(torch.linalg.matrix_norm(box_float))
        for i in range(len(boxes)):
            # boxi_float = torch.tensor(boxes[i], dtype=torch.float)
            corners_i = [[int(boxes[i][0,0]),int(boxes[i][0,1])], [int(boxes[i][0,2]),int(boxes[i][0,1])],
                          [int(boxes[i][0,2]),int(boxes[i][0,3])], [int(boxes[i][0,0]),int(boxes[i][0,3])]]
            # print(torch.linalg.matrix_norm(boxi_float-box_float))
            polygon_i = Polygon(corners_i)
            overlap = polygon.intersection(polygon_i).area / polygon.union(polygon_i).area
            if overlap > 0.3:
                if scores[i] > score:
                    break
                else:
                    boxes.pop(i)
                    labels.pop(i)
                    scores.pop(i)
                    scores.append(score)
                    labels.append(label)
                    boxes.append(box)
                    if depthImg_rcvd:
                        positions.pop(i)
                        positions.append(object_position)
                    break
            else:
                if score > 0.8:
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
        image = draw_bounding_boxes(image, boxes, width=5,
                          colors="green",labels=labels,
                          fill=True,font="/home/robot/Downloads/16020_FUTURAM.ttf",font_size=100)
        
    i = 1
    # transform and publish poses
    for pos in positions:
        pose = PoseStamped()
        pose.header.frame_id = "camera_color_optical_frame"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        try:
            transform = tf_buffer.lookup_transform("map", pose.header.frame_id, pose.header.stamp, rospy.Duration(2))
            pose_out = tf2_geometry_msgs.do_transform_pose(pose, transform)
            rospy.loginfo("Publishing pose")
            posePub.publish(pose_out)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            return
        
        # publish transform
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = "map"
        t.child_frame_id = "detection"+str(i)

        t.transform.translation = pose_out.pose.position
        try:
            map_to_base = tf_buffer.lookup_transform("map", "base_link", msg.header.stamp, timeout=rospy.Duration(2))
            rotation = np.array((map_to_base.transform.rotation.x, map_to_base.transform.rotation.y, map_to_base.transform.rotation.z, map_to_base.transform.rotation.w))
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(rotation)
            yaw = yaw + math.pi
            quat = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            t.transform.rotation.w = 1
        
        tfbroadcaster.sendTransform(t)

        i+=1

    
    pubImg = rnp.msgify(Image,image.permute(1,2,0).numpy(),encoding='rgb8')
    
    imgPub.publish(pubImg)
    errPub.publish(errors)


def cloudCB(msg):
    # Convert ROS -> Open3D
    cloud = o3drh.rospc_to_o3dpc(msg)

    # Downsample the point cloud to 5 cm
    ds_cloud = cloud.voxel_down_sample(voxel_size=0.01)

    # Convert Open3D -> NumPy
    points = np.asarray(ds_cloud.points)
    print("Points size: ",points.shape)
    colors = np.asarray(ds_cloud.colors)

    # sample color of ground
    # Min z: 0.25438890357812244, min x: -1.4876816272735596, min y: -19.418909072875977, max y: 0.36200428009033203
    min_dist = min(points[:,2])
    min_x = min(points[:,0])
    min_y = min(points[:,1])
    max_y = max(points[:,1])
    # print("Min z: {}, min x: {}, min y: {}, max y: {}".format(min_dist, min_x, min_y, max_y))
    cloud_ground = ds_cloud.select_by_index([i for i,p in enumerate(points) if p[2] < 0.5 and p[1] > 0.07])
    ground_color = np.mean(cloud_ground.colors, axis=0)
    print("Size ground points: ",np.asarray(cloud_ground.points).shape)

    # filter out ground
    nogroundCloud = ds_cloud.select_by_index([i for i,c in enumerate(colors) if
                                               ((c[0]<ground_color[0]-0.1) or (c[0]>ground_color[0]+0.1)) and
                                                ((c[1]<ground_color[1]-0.1) or (c[1]>ground_color[1]+0.1)) and
                                                 ((c[2]<ground_color[2]-0.1) or (c[2]>ground_color[2]+0.1))])
    noGroundPoints = np.asarray(nogroundCloud.points)

    

    reducedCloud = nogroundCloud.select_by_index([i for i, p in enumerate(noGroundPoints) if
                                                  (np.sqrt(p[0]**2 + p[2]**2) < 1) and
                                                   (np.sqrt(p[0]**2 + p[2]**2) > 0.2) and
                                                    (p[1] < 0.07) and (p[1] > -0.1)])

    # Convert Open3D -> ROS and publish reduced cloud
    out_msg = o3drh.o3dpc_to_rospc(reducedCloud)
    out_msg.header = msg.header
    cloudPub.publish(out_msg)

    points_filtered = np.array(reducedCloud.points)
    print(points_filtered.shape)
    if points_filtered.shape[0] != 0 and points_filtered.shape[0] < 30:
        averageVals = np.zeros((3,1))

        averageVals[0] = np.sum(points_filtered[:,0])/points_filtered.shape[0]
        averageVals[1] = np.sum(points_filtered[:,1])/points_filtered.shape[0]
        averageVals[2] = np.sum(points_filtered[:,2])/points_filtered.shape[0]

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = averageVals[0]
        pose.pose.position.y = averageVals[1]
        pose.pose.position.z = averageVals[2]

        try:
            transform = tf_buffer.lookup_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(2))
            pose_out = tf2_geometry_msgs.do_transform_pose(pose, transform)
            rospy.loginfo("Publishing pose")
            posePub.publish(pose_out)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            return

        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = "map"
        t.child_frame_id = "detection"

        t.transform.translation = pose_out.pose.position
        try:
            map_to_base = tf_buffer.lookup_transform("map", "base_link", msg.header.stamp, timeout=rospy.Duration(2))
            rotation = np.array((map_to_base.transform.rotation.x, map_to_base.transform.rotation.y, map_to_base.transform.rotation.z, map_to_base.transform.rotation.w))
            roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(rotation)
            yaw = yaw + math.pi
            quat = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            t.transform.rotation.w = 1
        
        tfbroadcaster.sendTransform(t)
    


if __name__=="__main__":
    rospy.init_node("detection")


    detectionModel = utils.load_model(detector.Detector(),"/home/robot/models/working_model/index.pt", device="cuda")
    detectionModel.eval()
    detectionModel = detectionModel.to(DEVICE)

    im = pil.open("/home/robot/Downloads/frame0008.jpg") 


    bridge = CvBridge()

    imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB,queue_size=10, buff_size=2**24)
    # cloudPub = rospy.Publisher("/detection/pointcloud", PointCloud2, queue_size=10)
    # pointCloudSub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, cloudCB)
    posePub = rospy.Publisher("/detection/pose", PoseStamped, queue_size=10)
    imgPub = rospy.Publisher("detection/overlaid_bbs", Image, queue_size=10)
    errPub = rospy.Publisher("/detection/bb_error", centerpointArray, queue_size=10)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()

    # Load model
    # detectionModel = utils.load_model(detector.Detector(),"~/RoboticsProj_VisionModel/models/det_2023-02-13_17-30-19-206874.pt", device="gpu")

    

    # detectionModel.eval()
    rospy.spin()