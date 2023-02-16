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
    with open("/home/robot/dd2419_ws/src/localization/src/exampleworkspace.tsv", "r") as f:
        for line in f:
            l = line.split("\t")
            points.append(l)
    
    polygon = Marker()
    polygon.header.frame_id = "arucomap"
    polygon.header.stamp = mesg.header.stamp
    polygon.ns = "aruco"
    polygon.id = mark.id
    polygon.type = Marker.LINE_STRIP
    for i in range(1,len(points)):
        polygon.points.append(Point(float(points[i][0]),float(points[i][1]), -t.transform.translation.z))
    polygon.points.append(Point(float(points[1][0]), float(points[1][1]),-t.transform.translation.z))
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
    
    for mark in markers:
        
        if mark.id == 2:
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
            yaw = yaw - math.pi/2
            q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
            
            
            
           
            
            
            
            if firsttime:
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
                

                #r.sleep()
            else:
                # if it sees the origin again compare where in the map we see the marker to where its first pose was in the map, the difference is the odom offset
                # should be done only with no angular movement
                if (angvel<0.1 and angvel>-0.1):
                    diffpose = PoseStamped()
                    diffpose.header.frame_id = "map"
                    diffpose.header.stamp = mesg.header.stamp
                    # diffpose.pose.position.x =  new_aruco_pose.pose.position.x - first_pose.pose.position.x
                    # diffpose.pose.position.y = new_aruco_pose.pose.position.y - first_pose.pose.position.y
                    # diffpose.pose.position.z = new_aruco_pose.pose.position.z - first_pose.pose.position.z
                    anglelist2 = [first_pose.pose.orientation.x, first_pose.pose.orientation.y, first_pose.pose.orientation.z,first_pose.pose.orientation.w]
                    roll2,pitch2,yaw2 = tf_conversions.transformations.euler_from_quaternion(anglelist2)
                    yaw = yaw - yaw2
                    try:
                        transform2 = tf_buffer.lookup_transform("map", "odom", latestupdate, rospy.Duration(2))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.loginfo(e)
                        continue
                    t3.header.frame_id = "map"
                    t3.child_frame_id = "odom"
                    t3.header.stamp = mesg.header.stamp
                    t3.transform.translation.x = transform2.transform.translation.x
                    t3.transform.translation.y = transform2.transform.translation.y 
                    t3.transform.translation.z = transform2.transform.translation.z
                    anglelist3 = [transform2.transform.rotation.x, transform2.transform.rotation.y, transform2.transform.rotation.z,transform2.transform.rotation.w]
                    roll3,pitch3,yaw3 = tf_conversions.transformations.euler_from_quaternion(anglelist3)
                    yaw3 = yaw3 - yaw
                    q3 = tf_conversions.transformations.quaternion_from_euler(roll3, pitch3, yaw3)
                    t3.transform.rotation.x = q3[0]
                    t3.transform.rotation.y = q3[1]
                    t3.transform.rotation.z = q3[2]
                    t3.transform.rotation.w = q3[3]
                    #br.sendTransform(t3)
                    transform = tf_buffer.lookup_transform("odom", "camera_link", latestupdate, rospy.Duration(2))
                    new_aruco_pose = tf2_geometry_msgs.do_transform_pose(arucopose, transform)
                    new_aruco_pose = tf2_geometry_msgs.do_transform_pose(new_aruco_pose, t3)
                    diffpose.pose.position.x =  new_aruco_pose.pose.position.x - first_pose.pose.position.x
                    diffpose.pose.position.y = new_aruco_pose.pose.position.y - first_pose.pose.position.y
                    diffpose.pose.position.z = new_aruco_pose.pose.position.z - first_pose.pose.position.z
                    t3.transform.translation.x = transform2.transform.translation.x - diffpose.pose.position.x
                    t3.transform.translation.y = transform2.transform.translation.y - diffpose.pose.position.y
                    t3.transform.translation.z = transform2.transform.translation.z - diffpose.pose.position.z
                    br.sendTransform(t3)
                    latestupdate = mesg.header.stamp



if __name__ == '__main__':
    rospy.init_node('aruco_node')
    first_pose.header.stamp=rospy.Time.now()
    aruco_sub = rospy.Subscriber('/aruco/markers', MarkerArray, aruco_callback)
    vel_sub = rospy.Subscriber('/imu/data', Imu, vel_callback)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    st = tf2_ros.StaticTransformBroadcaster()
    #The while loop is necessary to so that global variables are not reset. then we have to continuouslty publish teh transform between map and odom so the point cloud works, a better way could be to use a static transform.
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (t3.header.stamp.to_sec() - now.to_sec()<= -1):
            t3.header.stamp = now
            br.sendTransform(t3)
            rospy.loginfo("send transform")
        elif (t3.header.stamp.to_nsec()*10**(-9) - now.to_nsec()*10**(-9) <= -0.1):
            t3.header.stamp = now
            br.sendTransform(t3)
        continue
    rospy.spin()
