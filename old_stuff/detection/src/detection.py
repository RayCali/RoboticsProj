#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np

pub = None


def cloud_callback(msg: PointCloud2):
    # Convert ROS -> Open3D
    cloud = o3drh.rospc_to_o3dpc(msg)
    
    # Downsample the point cloud to 5 cm
    ds_cloud = cloud.voxel_down_sample(voxel_size=0.05)

    # Convert Open3D -> NumPy
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)

    for i in range(0,len(colors)):
        if colors[i,0] >= 0.8 and colors[i,1] <= 0.4 and colors[i,2]<= 0.4 and 0 < points[i,0]<0.5 and -0.6< points[i,1]<0.6 and 0< points[i,2]<0.6:
            print('Red')

        if colors[i,0] < 0.4 and colors[i,1] > 0.4 and colors[i,2]< 0.4 and 0 < points[i,0]<0.5 and -0.6< points[i,1]<0.6 and 0< points[i,2]<0.6:
            print('Green')
 
   
    # Convert Open3D -> ROS
    out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
    out_msg.header = msg.header
    pub.publish(out_msg)


rospy.init_node('detection')
sub_goal = rospy.Subscriber(
    '/camera/depth/color/points', PointCloud2, cloud_callback)

pub = rospy.Publisher('/camera/depth/color/ds_points', PointCloud2, queue_size=1)

if __name__ == '__main__':
    rospy.spin()