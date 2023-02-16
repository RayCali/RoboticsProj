#!/usr/bin/python3

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import ros_numpy as rnp
import torch
from scipy.ndimage import binary_erosion, binary_dilation
from sensor_msgs.msg import Image, PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
from geometry_msgs.msg import PoseStamped, TransformStamped
import utils, detector

FOCAL_LENGTH = 1.93/1000 # focal lenth in m
BASELINE = 50/1000 # baseline in m (distance between the two infrared cams)
BASELINE_1 = 65/1000 # distance in m between color cam and right infrared cam
BASELINE_2 = 15/1000 # distance in m between color cam and left infrared cam

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

# def depthCB(msg):
#     # transform depth image to np image 
#     np_image = rnp.numpify(msg)

#     # the image is greyscale with intensity vals from 0 to 255
#     # 0 -> white, far away; 255 -> black, close
#     # make simple distance filter to find points that are closer
#     filter = np.zeros(np.shape(np_image))
#     filter[(np_image<200)&(np_image>100)] = 1

#     structElem = np.ones((3,3))

#     openedFilter = openFilter(filter, structElem)

#     finalFilter = closeFilter(openedFilter,structElem)

def cloudCB(msg):
    # Convert ROS -> Open3D
    cloud = o3drh.rospc_to_o3dpc(msg)

    # Downsample the point cloud to 5 cm
    ds_cloud = cloud.voxel_down_sample(voxel_size=0.05)

    # Convert Open3D -> NumPy
    points = np.asarray(ds_cloud.points)
    #colors = np.asarray(ds_cloud.colors)

    filterMask = np.zeros(len(points[:,1]))
    filterMask[(np.sqrt(points[:,0]**2 + points[:,1]**2) < 0.8) &
               (np.sqrt(points[:,0]**2 + points[:,1]**2) > 0.3) &
               (points[:,2] > 0.1)] = 1

    structElem = np.ones((3,3))

    openedFilter = openFilter(filterMask, structElem)

    closedFilter = closeFilter(openedFilter,structElem)

    finalFilter = np.zeros(np.shape(points))
    finalFilter[:,0] = finalFilter[:,1] = finalFilter[:,2] = closedFilter

    points_filtered = np.multiply(points, finalFilter)

    nonZeros = [val for ind, val in enumerate(points_filtered) if (val[0]**2 + val[1]**2 + val[2]**2) != 0]
    nonZeros = np.array(nonZeros)

    averageVals = np.zeros((3,1))

    averageVals[0] = np.sum(nonZeros[0,:])/len(nonZeros[0,:])
    averageVals[1] = np.sum(nonZeros[1,:])/len(nonZeros[1,:])
    averageVals[2] = np.sum(nonZeros[2,:])/len(nonZeros[2,:])


    # for i in range(len(points[:,1])):
    #     dist = np.sqrt(points[i,0]**2 + points[i,1]**2)
    #     height = points[i,2]

    #     # Filtering points by distance and height from the ground
    #     if (dist < 0.8) and (height > -0.1):
    #         filterMask[i] = 1

    pose = PoseStamped()
    pose.header = msg.header
    pose.pose.position.x = averageVals[0]
    pose.pose.position.y = averageVals[1]
    pose.pose.position.z = averageVals[2]

    try:
        transform = tf_buffer.lookup_transform("map", pose.header.frame_id, msg.header.stamp, rospy.Duration(2))
        pose_out = tf2_geometry_msgs.do_transform_pose(pose, transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)

    t = TransformStamped()
    t.header = msg.header
    t.header.frame_id = "map"
    t.child_frame_id = "detection"

    t.transform.translation = pose_out.pose.position
    t.transform.rotation.w = 1
    #transform = tf_buffer.lookup_transform("map", "base_link", msg.header.stamp, rospy.Duration(2))
    tfbroadcaster.sendTransform(t)

    rospy.loginfo("Publishing pose")
    posePub.publish(pose_out)


    
def openFilter(filter, structElem):
    erosion = binary_erosion(filter)
    return binary_dilation(erosion)


def closeFilter(filter, structElem):
    dilation = binary_dilation(filter)
    return binary_erosion(dilation)
    


if __name__=="__main__":
    rospy.init_node("detection")

    #imageSub = rospy.Subscriber("/camera/color/image_raw", Image, imageCB)
    pointCloudSub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, cloudCB)
    posePub = rospy.Publisher("/detection/pose", PoseStamped, queue_size=10)
    imgPub = rospy.Publisher("detection/overlaid_bbs", Image, queue_size=10)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()

    # Load model
    # detectionModel = utils.load_model(detector.Detector(),"~/RoboticsProj_VisionModel/models/det_2023-02-13_17-30-19-206874.pt", device="gpu")

    # detectionModel.eval()
    rospy.spin()