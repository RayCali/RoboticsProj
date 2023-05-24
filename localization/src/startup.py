#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import datetime
from aruco_msgs.msg import MarkerArray
import math
from visualization_msgs.msg import Marker
import os
from robp_msgs.msg import DutyCycles
listener=None
markers = None
tf_buffer = None
br =None
st=None
firsttime=True
first_pose=PoseStamped()
first_pose.pose.position.x=0
first_pose.pose.position.y=0
first_pose.pose.position.z=0
t3=TransformStamped()
t3.header.frame_id = "map"
t3.child_frame_id = "odom"
t3.transform.translation.x=0
t3.transform.translation.y=0
t3.transform.translation.z=0
t3.transform.rotation.x=0
t3.transform.rotation.y=0
t3.transform.rotation.z=0
t3.transform.rotation.w=1
latestupdate=0
angvel = 0
from sensor_msgs.msg import Imu
marker_pub = rospy.Publisher("/boundaries", Marker, queue_size=10)



def vel_callback(mesg:Imu):
    global angvel
    angvel=mesg.angular_velocity.z



def createWorkspace(t:TransformStamped,mesg,mark):
    global marker_pub
    rospy.loginfo(os.getcwd())
    points = []

    #with open("/home/robot/dd2419_ws/src/localization/src/exampleworkspace.tsv", "r") as f:
    with open("/home/robot/dd2419_ws/src/localization/src/demoworkspace.tsv", "r") as f:
        for line in f:
            line = line.strip("\n")
            l = line.split("\t")
            points.append(l)
    
    polygon = Marker()
    polygon.header.frame_id = "arucomap"
    polygon.header.stamp = mesg.header.stamp
    polygon.ns = "aruco"
    polygon.id = mark.id
    polygon.type = Marker.LINE_STRIP

    #for i in range(1,len(points)):
    #    polygon.points.append(Point(float(points[i][0])-4.02,float(points[i][1])-1.21, -t.transform.translation.z))
    #polygon.points.append(Point(float(points[1][0])-4.02, float(points[1][1])-1.21,-t.transform.translation.z))
    x = -1
    y = -1
    for i in range(1,len(points)):
        polygon.points.append(Point(float(points[i][0])-x,float(points[i][1])-y, -t.transform.translation.z))
    polygon.points.append(Point(float(points[1][0])-x, float(points[1][1])-y,-t.transform.translation.z))
    polygon.scale.x = 0.03
    polygon.color.a = 1.0
    polygon.color.r = 1.0
    polygon.color.g = 0.0
    polygon.color.b = 0.0
    marker_pub.publish(polygon)


def aruco_callback(mesg):
    global markers, listener, tf_buffer,br,firsttime,first_pose,t3, st, marker_pub, latestupdate,angvel
    markers =mesg.markers
    #r=r=rospy.Rate(10)
    
    for mark in markers:
        
        if mark.id == 500:
            if firsttime:
                t = TransformStamped()
                t.header.frame_id = "map"
                t.child_frame_id = "odom"
                t.header.stamp = mesg.header.stamp
                t.transform.translation.x = 0
                t.transform.translation.y = 0
                t.transform.translation.z = 0
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1
                latestupdate=mesg.header.stamp
                br.sendTransform(t)
                arucopose = PoseStamped()
                arucopose.header.frame_id="/camera_link"
                arucopose.header.stamp=mesg.header.stamp
                arucopose.pose.position.x = mark.pose.pose.position.x 
                arucopose.pose.position.y = mark.pose.pose.position.y 
                arucopose.pose.position.z = mark.pose.pose.position.z 
                arucopose.pose.orientation = mark.pose.pose.orientation
                rate = rospy.Rate(10.0)
                try:
                    transform = tf_buffer.lookup_transform("map", "camera_link", latestupdate, rospy.Duration(2))
                    new_aruco_pose = tf2_geometry_msgs.do_transform_pose(arucopose, transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.loginfo(e)
                    continue
                # transform = tf_buffer.lookup_transform("map", "camera/depth/color/points", mesg.header.stamp, rospy.Duration(1.0))
                # new_aruco_pose = tf2_geometry_msgs.do_transform_pose(arucopose, transform)
                anglelist = [new_aruco_pose.pose.orientation.x, new_aruco_pose.pose.orientation.y, new_aruco_pose.pose.orientation.z,new_aruco_pose.pose.orientation.w]
                roll,pitch,yaw = tf_conversions.transformations.euler_from_quaternion(anglelist)
                yaw = yaw
                q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
                #if it sees origin for the first time create a static transform for the origin and save its pose for future updates of odom
                statictransform = TransformStamped()
                statictransform.header.frame_id = "map"
                statictransform.child_frame_id = "arucomap"
                statictransform.header.stamp = mesg.header.stamp
                statictransform.transform.translation.x = new_aruco_pose.pose.position.x
                statictransform.transform.translation.y = new_aruco_pose.pose.position.y
                statictransform.transform.translation.z = new_aruco_pose.pose.position.z
                
                statictransform.transform.rotation.x =  q[0]      #new_aruco_pose.pose.orientation.x
                statictransform.transform.rotation.y =  q[1]      #new_aruco_pose.pose.orientation.y
                statictransform.transform.rotation.z =  q[2]      #new_aruco_pose.pose.orientation.z
                statictransform.transform.rotation.w =  q[3]      #new_aruco_pose.pose.orientation.w
                st.sendTransform(statictransform)
                first_pose.pose.position.x = new_aruco_pose.pose.position.x
                first_pose.pose.position.y = new_aruco_pose.pose.position.y
                first_pose.pose.position.z = new_aruco_pose.pose.position.z
                first_pose.pose.orientation.x = q[0]
                first_pose.pose.orientation.y = q[1]
                first_pose.pose.orientation.z = q[2]
                first_pose.pose.orientation.w = q[3]
                firsttime=False
                createWorkspace(statictransform, mesg,mark)
    



if __name__ == '__main__':
    rospy.init_node('aruco_node')
    first_pose.header.stamp=rospy.Time.now()
    aruco_sub = rospy.Subscriber('/aruco_500/aruco/markers', MarkerArray, aruco_callback)
    vel_sub = rospy.Subscriber('/imu/data', Imu, vel_callback)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    st = tf2_ros.StaticTransformBroadcaster()
    #The while loop is necessary to so that global variables are not reset. then we have to continuouslty publish teh transform between map and odom so the point cloud works, a better way could be to use a static transform.
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (t3.header.stamp.to_sec() - now.to_sec()<= -0.1):
            currenttransform = tf_buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(2))
            if t3.transform.translation.x != currenttransform.transform.translation.x or t3.transform.translation.y != currenttransform.transform.translation.y or t3.transform.translation.z != currenttransform.transform.translation.z or t3.transform.rotation.x != currenttransform.transform.rotation.x or t3.transform.rotation.y != currenttransform.transform.rotation.y or t3.transform.rotation.z != currenttransform.transform.rotation.z or t3.transform.rotation.w != currenttransform.transform.rotation.w:
                t3 = currenttransform
                t3.header.stamp = now
                st.sendTransform(t3)
                rospy.loginfo("send transform")
        continue
    rospy.spin()
