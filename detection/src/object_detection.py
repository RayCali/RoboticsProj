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



FOCAL_LENGTH = 1.93/1000 # focal lenth in m
BASELINE = 50/1000 # baseline in m (distance between the two infrared cams)
BASELINE_1 = 65/1000 # distance in m between color cam and right infrared cam
BASELINE_2 = 15/1000 # distance in m between color cam and left infrared cam
CENTERIMG_X = 640 # center of image in x direction

def imageCB(msg):
    #imgPub.publish(msg)
    # msg is of type Image, convert to torch tensor
    
    np_image = rnp.numpify(msg) # shape: (720,1280,3)
    
    image = transforms.ToTensor()(np_image) # shape: (3,720,1280)
    
    image = image.unsqueeze(0)
        

    inference = detectionModel() # size: (15,20,5)
    
    bbs = detectionModel.decode_output(inference, threshold=0.7)[0]
    bbxArrayMsg = boundingboxArray()
    image = torch.from_numpy(np_image).permute(2,0,1)
    for bbx in bbs:
        x, y, width, height, score = bbx["x"], bbx["y"], bbx["width"], bbx["height"], bbx["score"]
        bbxMsg = boundingboxMsg()
        bbxMsg.x = x
        bbxMsg.y = y
        bbxMsg.width = width
        bbxMsg.height = height
        bbxMsg.score = score
        bbxArrayMsg.bbs.append(bbxMsg)
    
        
        # extract center position of box
        centerbbx_x = x+int(width/2)
        centerbb_y = y-int(height/2)
        # deviation of bounding box from center (only in x)
        error_x = centerbbx_x - CENTERIMG_X

        # overlay box on image
        #X = np.arange(x, x+width)
        #Y = np.arange(y, y-height, step=-1)
        box = torch.tensor([x,y-height,x+width,y], dtype=torch.int).unsqueeze(0)
        image = draw_bounding_boxes(image, box, width=5,
                          colors="green", 
                          fill=True)
        #image = transforms.ToPILImage()(image)
        #loginfo(box)
        #plt.imshow(image)
        #plt.show()
        #return
        #image = rnp.numpify(msg)
        #image[Y,X,0] = 255
        #image[Y,X,1] = 0
        #image[Y,X,2] = 0
    
    #pubImg = rnp.msgify(Image,rnp.numpify(msg),encoding='rgb8')
    loginfo(image.size())
    pubImg = rnp.msgify(Image,image.permute(1,2,0).numpy(),encoding='rgb8')
    
    imgPub.publish(pubImg)
    return
    error_msg = Vector3Stamped()
    error_msg.header.stamp = msg.header.stamp
    error_msg.header.frame_id = msg.header.frame_id
    error_msg.vector.x = error_x

    errPub.publish(error_msg)


def cloudCB(msg):
    return
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
    imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB)
    cloudPub = rospy.Publisher("/detection/pointcloud", PointCloud2, queue_size=10)
    pointCloudSub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, cloudCB)
    posePub = rospy.Publisher("/detection/pose", PoseStamped, queue_size=10)
    imgPub = rospy.Publisher("detection/overlaid_bbs", Image, queue_size=10)
    errPub = rospy.Publisher("/detection/bb_error", Vector3Stamped, queue_size=10)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()

    # Load model
    # detectionModel = utils.load_model(detector.Detector(),"~/RoboticsProj_VisionModel/models/det_2023-02-13_17-30-19-206874.pt", device="gpu")

    

    # detectionModel.eval()
    rospy.spin()