#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import tf2_msgs.msg
import tf
from geometry_msgs.msg import PoseStamped
import ros_numpy
import numpy as np
pub = None


def cloud_callback(msg: PointCloud2):
    # Convert between ROS -> Open3D
    rate = rospy.Rate(10)
    pub.publish(msg)
    rate.sleep()
    
rospy.init_node('PCtoLS')
sub_goal = rospy.Subscriber('/camera/depth/image_rect_raw', PointCloud2, cloud_callback)


pub = rospy.Publisher('image', PointCloud2, queue_size=1)
pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

if __name__ == '__main__':
    rospy.spin()