#!/usr/bin/python3
import rospy
import sys
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from aruco_msgs.msg import MarkerArray
import math
from visualization_msgs.msg import Marker
import numpy as np
from geometry_msgs.msg import PoseStamped

x=0
y=0
yaw=0
br= None
tfBuffer = None
listener = None
R = np.identity(3)*0.01
Q = np.identity(2)*0.01
H = np.array([[-1,0,0,1,0],[0,-1,0,0,1]])
landmarks = 1
firsttime0 = True
firsttime1 = True
firsttime2 = True
mu_slam = np.zeros(3+2*landmarks)
P_up = np.zeros([3,3+2*landmarks])
p_downleft = np.zeros([2*landmarks,3])
p_downright = np.identity(2*landmarks)*(10**9)
p_down = np.concatenate((p_downleft,p_downright),axis=1)
P = np.concatenate((P_up,p_down),axis=0)
Fxmapcols = np.zeros([3,2*landmarks])
Fxposcols = np.identity(3)
Fx = np.concatenate((Fxmapcols,Fxposcols),axis=1)
G = np.zeros([3+2*landmarks,3+2*landmarks])
G[3:3+2*landmarks,3:3+2*landmarks]=np.identity(2*landmarks)
latestupdate = None
Landmarklist = []
#create a class with attributes: int id and int order
class Landmark:
    def __init__(self, id, order):
        self.id = id
        self.order = order
    def __str__(self):
        return "id: " + str(self.id) + " order: " + str(self.order)
    def __repr__(self):
        return self.__str__()







def predict_callback(msg:Twist):
    global x,y,yaw,Fx,mu_slam,P,G, tfBuffer
# find the transform between bas and map, set that to mu_slam calculate G from the message

    
    
def update_callback(msg: MarkerArray):
    global x,y,yaw,Fx,mu_slam,P,G, tfBuffer, listener, firsttime0, firsttime1, firsttime2,R,br,latestupdate,Landmarklist,landmarks
    for mark in msg.markers:
        if mark.id!=500:
            time = rospy.Time(0)
            hasbeenseen=False
            transform = tfBuffer.lookup_transform('map', 'camera_link', time, rospy.Duration(1.0))
            markerpose = tf2_geometry_msgs.do_transform_pose(mark.pose, transform)
            currentid = 1000000
            currentorder = 1000000
            for landmarks in Landmarklist:
                if landmarks.id == mark.id:
                    hasbeenseen=True
                    currentid = landmarks.id
                    currentorder = landmarks.order
                    break
            if not hasbeenseen:
                if len(Landmarklist)<=landmarks:
                    Landmarklist.append(Landmark(mark.id,len(Landmarklist)))
                    currentorder = len(Landmarklist)-1
                    currentid = mark.id
                    mu_slam[currentorder*2 + 3],mu_slam[currentorder*2 + 4]=markerpose.pose.position.x,markerpose.pose.position.y
                    P[currentorder*2 + 3:currentorder*2 + 5,currentorder*2 + 3:currentorder*2 + 5]= P[0:2,0:2]
                    marktransform = TransformStamped()
                    marktransform.header.frame_id = "map"
                    marktransform.child_frame_id = "arucolandmark" + str(mark.id)
                    marktransform.header.stamp = msg.header.stamp
                    marktransform.transform.translation.x = markerpose.pose.position.x
                    marktransform.transform.translation.y = markerpose.pose.position.y
                    marktransform.transform.translation.z = markerpose.pose.position.z
                    anglelist = [markerpose.pose.orientation.x, markerpose.pose.orientation.y, markerpose.pose.orientation.z,markerpose.pose.orientation.w]
                    roll,pitch,yaw2 = tf_conversions.transformations.euler_from_quaternion(anglelist)
                    roll = roll - math.pi/2
                    yaw2 = yaw2 - math.pi/2
                    q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw2)
                    marktransform.transform.rotation.x =  q[0]
                    marktransform.transform.rotation.y =  q[1]     
                    marktransform.transform.rotation.z =  q[2]      
                    marktransform.transform.rotation.w =  q[3]      
                    br.sendTransform(marktransform)
                    firsttime0 = False
                    latestupdate = rospy.Time.now()
            else:
                addon = np.zeros([2,3+2*landmarks])
                addon[0, 3+2*currentorder] = 1
                addon[1, 4+2*currentorder] = 1
                Fxj = np.concatenate((Fx,addon),axis=0)
                
                
                #range bearing part
                delta = np.array([mu_slam[currentorder*2 + 3]-mu_slam[0],mu_slam[currentorder*2 + 4]-mu_slam[1]])
                q = np.matmul(np.transpose(delta),delta)
                zpredict = np.array([math.sqrt(q),math.atan2(delta[1],delta[0])-mu_slam[2]])
                H = 1/q*np.array([[-math.sqrt(q)*delta[0],-math.sqrt(q)*delta[1],0,math.sqrt(q)*delta[0],math.sqrt(q)*delta[1]],[delta[1],-delta[0],-q,-delta[1],delta[0]]])
                transformb = tfBuffer.lookup_transform('base_link', 'camera_link', time, rospy.Duration(1.0))
                markerposeb = tf2_geometry_msgs.do_transform_pose(mark.pose, transformb)
                delta = np.array([markerposeb.pose.position.x,markerposeb.pose.position.y])
                q = np.matmul(np.transpose(delta),delta)
                z = np.array([math.sqrt(q),math.atan2(delta[1],delta[0])])
                Hj = np.matmul(H,Fxj)
                
                
                
                K = np.matmul(np.matmul(P,np.transpose(Hj)),np.linalg.inv(np.matmul(np.matmul(Hj,P),np.transpose(Hj))+Q))

                mu = mu + np.matmul(K,z-zpredict)
                rospy.loginfo(np.matmul(K,z-zpredict))
                




                transformod = tfBuffer.lookup_transform('map', 'odom', time, rospy.Duration(1.0))
                anglelist = [transformod.transform.rotation.x, transformod.transform.rotation.y, transformod.transform.rotation.z,transformod.transform.rotation.w]
                roll,pitch,yaw2 = tf_conversions.transformations.euler_from_quaternion(anglelist)
                #yawnew = yaw2 + diff[2] 
                yawnew = yaw2 + np.matmul(K,z-zpredict)[2]
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, yawnew)
                newodom = TransformStamped()
                newodom.header.frame_id = "map"
                newodom.child_frame_id = "odom"
                newodom.header.stamp = msg.header.stamp
                newodom.transform.translation.x = np.matmul(K,z-zpredict)[0] + transformod.transform.translation.x
                newodom.transform.translation.y = np.matmul(K,z-zpredict)[1] + transformod.transform.translation.y
                newodom.transform.translation.z = 0
                newodom.transform.rotation.x =  q[0]
                newodom.transform.rotation.y =  q[1]
                newodom.transform.rotation.z =  q[2]
                newodom.transform.rotation.w =  q[3]
                br.sendTransform(newodom)




                latestupdate = rospy.Time.now()
                P = np.matmul(np.identity(3+2*landmarks)-np.matmul(K,Hj),P)
                marktransform = TransformStamped()
                marktransform.header.frame_id = "map"
                marktransform.child_frame_id = "arucolandmark"
                marktransform.header.stamp = msg.header.stamp
                marktransform.transform.translation.x = mu_slam[currentorder*2 + 3]
                marktransform.transform.translation.y = mu_slam[currentorder*2 + 4]
                marktransform.transform.translation.z = markerpose.pose.position.z
                marktransform.transform.rotation.x =  0      #markerpose.pose.orientation.x
                marktransform.transform.rotation.y =  0      #markerpose.pose.orientation.y
                marktransform.transform.rotation.z =  0      #markerpose.pose.orientation.z
                marktransform.transform.rotation.w =  1      #markerpose.pose.orientation.w
                br.sendTransform(marktransform)

                rospy.loginfo("updated")
        else:
            P[0:3,0:3].fill(0)


if __name__ == '__main__':
    rospy.init_node('ekf_slam')
    sub_goal = rospy.Subscriber('/predictedvel', Encoders, predict_callback)
    update = rospy.Subscriber('/aruco/markers', MarkerArray, update_callback)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(12000.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        continue
    
    rospy.spin()