#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from msg_srv_pkg.msg import objectPoseStampedLst
import tf_conversions
import tf2_ros
import tf2_geometry_msgs


if __name__ == "__main__":
    rospy.init_node('pickup_client')

    posePub = rospy.Publisher("/object_finalpose", PoseStamped, queue_size=1)
    
    test_pose = PoseStamped()
    test_pose.header.frame_id = "arm_base"
    test_pose.header.stamp = rospy.Time.now()
    test_pose.pose.position.x = 0.16
    test_pose.pose.position.y = 0.0
    test_pose.pose.position.z = -0.035
    test_pose.pose.orientation.x = 0.0
    test_pose.pose.orientation.y = 0.0
    test_pose.pose.orientation.z = 0.0
    test_pose.pose.orientation.w = 1.0
    posePub.publish(test_pose)

    while not rospy.is_shutdown():
        posePub.publish(test_pose)
