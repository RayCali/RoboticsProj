#!/usr/bin/python3

import os
import rospy
from rospy import loginfo
import tf2_ros
# import torch
# from torchvision import transforms
from global_explorer import getMostValuedCell
from typing import List
from config import SUCCESS, RUNNING, FAILURE
from visualization_msgs.msg import Marker as VMarker
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovariance, Pose, Quaternion
from aruco_msgs.msg import MarkerArray, Marker
from math import sqrt

def doUpdateOdom():
    t3=TransformStamped()
    t3.header.frame_id = "map"
    t3.child_frame_id = "odom"
    t3.header.stamp = rospy.Time.now()
    t3.transform.translation.x=0
    t3.transform.translation.y=0
    t3.transform.translation.z=0
    t3.transform.rotation.x=0
    t3.transform.rotation.y=0
    t3.transform.rotation.z=0
    t3.transform.rotation.w=1
    st.sendTransform(t3)

def doCreateWorkspace():
    points = []
    with open("/home/robot/local_ws/src/localization/src/exampleworkspace.tsv", "r") as f:
        for line in f:
            line = line.strip("\n")
            l = line.split("\t")
            points.append(l)
    
    polygon = VMarker()
    polygon.header.frame_id = "arucomap"
    polygon.header.stamp = rospy.Time.now()
    polygon.ns = "aruco"
    polygon.id = 500
    polygon.type = VMarker.LINE_STRIP
    for i in range(1,len(points)):
        polygon.points.append(Point(float(points[i][0])-4.02,float(points[i][1])-1.21, 0))
    polygon.points.append(Point(float(points[1][0])-4.02, float(points[1][1])-1.21, 0))
    polygon.scale.x = 0.03
    polygon.color.a = 1.0
    polygon.color.r = 1.0
    polygon.color.g = 0.0
    polygon.color.b = 0.0
    marker_pub.publish(polygon)

def doPublishMarkerArray():
    header = Header()
    header.seq = 500
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    markerarray = MarkerArray()
    markerarray.header = header
    marker = Marker()
    marker.header = header
    marker.id = 500
    marker.pose = PoseWithCovariance()
    marker.pose.pose = Pose()
    point = Point()
    point.x = 0.2
    marker.pose.pose.position = point
    orientation = Quaternion()
    orientation.w = 1
    marker.pose.pose.orientation = orientation
    markerarray.markers.append(marker)
    aruco_pub.publish(markerarray)

def doPublish500Marker():
    statictransform = TransformStamped()
    statictransform.header.frame_id = "map"
    statictransform.child_frame_id = "arucomap"
    statictransform.header.stamp = rospy.Time.now()
    statictransform.transform.translation.x = 1
    statictransform.transform.translation.y = 0
    statictransform.transform.translation.z = 0
    
    statictransform.transform.rotation.x =  0
    statictransform.transform.rotation.y =  0
    statictransform.transform.rotation.z =  -1/sqrt(2)
    statictransform.transform.rotation.w =  1/sqrt(2)
    st.sendTransform(statictransform)
    pass


if __name__ == "__main__":

    rospy.init_node("simulated_run")   
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    listener = tf2_ros.TransformListener(tf_buffer)
    aruco_pub = rospy.Publisher('/aruco_500/aruco/markers', MarkerArray, queue_size=1000)
    marker_pub = rospy.Publisher("/boundaries", VMarker, queue_size=1000)
    st = tf2_ros.StaticTransformBroadcaster()
    while not rospy.is_shutdown():
        #doUpdateOdom()
        doCreateWorkspace()
        doPublish500Marker()
        rospy.sleep(5)