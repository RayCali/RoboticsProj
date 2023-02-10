#!/usr/bin/env python3
import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_geometry_msgs
import tf_conversions
import tf2_ros
import math

def marker_callback(msg: MarkerArray):
    #rospy.loginfo('New encoder received:\n%s', msg)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    stamper = msg.header.stamp
    # timeout = rospy.Duration(0.5)
    try:
        trans = tfBuffer.lookup_transform('map','camera_link', stamper) #rospy.Time()
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return

    pose_stamp = PoseStamped()
    pose_stamp.pose.position.x = msg.markers[0].pose.pose.position.x
    pose_stamp.pose.position.y = msg.markers[0].pose.pose.position.y
    pose_stamp.pose.position.z = msg.markers[0].pose.pose.position.z
    id = msg.markers[0].id
    rospy.loginfo(id)

    pose_stamp.pose.orientation.x = msg.markers[0].pose.pose.orientation.x
    pose_stamp.pose.orientation.y = msg.markers[0].pose.pose.orientation.y
    pose_stamp.pose.orientation.z = msg.markers[0].pose.pose.orientation.z
    pose_stamp.pose.orientation.w = msg.markers[0].pose.pose.orientation.w

    q = tf_conversions.transformations.euler_from_quaternion([
        pose_stamp.pose.orientation.x ,
        pose_stamp.pose.orientation.y,
        pose_stamp.pose.orientation.z,
        pose_stamp.pose.orientation.w,

    ])
    if id ==2:
        q  = (q[0]- math.pi/2, q[1], q[2] - math.pi/2)

    if id ==1:
        q  = (q[0] + math.pi/2, q[1]+ math.pi, q[2] - math.pi/2)

    q = tf_conversions.transformations.quaternion_from_euler(q[0], q[1], q[2])
    

    # Broadcaster
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "map"
    t.child_frame_id = "aruco/detected%s" % str(id) #"aruco/detectedX"

    transformed = tf2_geometry_msgs.do_transform_pose(pose_stamp,trans)
    t.transform.translation.x = transformed.pose.position.x
    t.transform.translation.y = transformed.pose.position.y
    t.transform.translation.z = transformed.pose.position.z

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)
    
rospy.init_node('display_markers')
sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)

if __name__ == '__main__':
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.spin()