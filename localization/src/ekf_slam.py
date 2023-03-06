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
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
import math
import numpy as np
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker
yaw = 0
br= None
tfBuffer = None
listener = None
R = np.identity(3)*0.01
Q = np.identity(2)*0.01
H = np.array([[-1,0,0,1,0],[0,-1,0,0,1]])
landmarks = 3
mu_slam = np.zeros(3+2*landmarks)
P_up = np.zeros([3,3+2*landmarks])
p_downleft = np.zeros([2*landmarks,3])
p_downright = np.identity(2*landmarks)*(10**9)
p_down = np.concatenate((p_downleft,p_downright),axis=1)
P = np.concatenate((P_up,p_down),axis=0)
Fxmapcols = np.zeros([3,2*landmarks])
Fxposcols = np.identity(3)
Fx = np.concatenate((Fxposcols,Fxmapcols),axis=1)
G = np.zeros([3+2*landmarks,3+2*landmarks])
G[3:3+2*landmarks,3:3+2*landmarks]=np.identity(2*landmarks)
latestupdate = None
Landmarklist = []
marker_pub = rospy.Publisher("/covariances", MarkerArray, queue_size=10)
#create a class with attributes: int id and int order
class Landmark:
    def __init__(self, id, order):
        self.id = id
        self.order = order
    def __str__(self):
        return "id: " + str(self.id) + " order: " + str(self.order)
    def __repr__(self):
        return self.__str__()
    def getid(self):
        return self.id



def updaterviz():
    global mu_slam, P, br, landmarks, Landmarklist
    # visualize the covariance ellipses of robot and landmarks    
    rate = rospy.Rate(10)
    markerarray = MarkerArray()
    for landmark in Landmarklist:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "covariance"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = mu_slam[landmark.order*2+3]
        marker.pose.position.y = mu_slam[landmark.order*2+4]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        (eigvals, eigvecs) = np.linalg.eig(P[landmark.order*2+3:landmark.order*2+5,landmark.order*2+3:landmark.order*2+5])
        marker.scale.x = eigvals[0]*2
        marker.scale.y = eigvals[1]*2
        marker.scale.z = 0.1
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.id = landmark.id
        markerarray.markers.append(marker)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "covariance"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = mu_slam[0]
    marker.pose.position.y = mu_slam[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    (eigvals, eigvecs) = np.linalg.eig(P[0:2,0:2])
    marker.scale.x = eigvals[0]*2
    marker.scale.y = eigvals[1]*2
    marker.scale.z = 0.1
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.id = 400
    markerarray.markers.append(marker)
    marker_pub.publish(markerarray)
    rate.sleep()
def updatervizpos():
    global mu_slam, P, br, landmarks, Landmarklist
    rate = rospy.Rate(40)
    markerArray = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "covariance"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = mu_slam[0]
    marker.pose.position.y = mu_slam[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0


    (eigvals, eigvecs) = np.linalg.eig(P[0:2,0:2])
    marker.scale.x = eigvals[0]*2
    marker.scale.y = eigvals[1]*2
    marker.scale.z = 0.1
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.id = 400
    markerArray.markers.append(marker)
    marker_pub.publish(markerArray)
    rate.sleep()

    
    
    
def predict_callback(msg:Twist):
    global yaw,Fx,mu_slam,P,G, tfBuffer, yaw
    dt = 1/90.9
    v = msg.linear.x
    w = msg.angular.z
    transformblmap = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    mu_slam[0] = transformblmap.transform.translation.x
    mu_slam[1] = transformblmap.transform.translation.y
    anglelist = [transformblmap.transform.rotation.x, transformblmap.transform.rotation.y, transformblmap.transform.rotation.z,transformblmap.transform.rotation.w]
    roll,pitch,yaw2 = tf_conversions.transformations.euler_from_quaternion(anglelist)
    mu_slam[2] = yaw2
    yaw = yaw + w*dt
    Gx = np.array([[1,0,-v*dt*math.sin(yaw)],[0,1,v*dt*math.cos(yaw)],[0,0,1]])
    G[0:3,0:3]=Gx
    rospy.loginfo(v)
    rospy.loginfo(w)
    if w<0.001 and w>-0.001  and v<1e-5:
        R = np.zeros((3,3))
    else:
        R = np.identity(3)*0.01
    P = np.matmul(np.matmul(G,P),np.transpose(G)) + np.matmul(np.matmul(np.transpose(Fx),R),Fx)
    updatervizpos()
# find the transform between bas and map, set that to mu_slam calculate G from the message


    
    
def update_callback(msg: MarkerArray):
    global yaw,Fx,mu_slam,P,G, tfBuffer, listener, firsttime0, firsttime1, firsttime2,R,br,latestupdate,Landmarklist,landmarks
    for mark in msg.markers:
        if mark.id!=500:
            time = rospy.Time(0)
            hasbeenseen=False
            transform = tfBuffer.lookup_transform('map', 'camera_link', time, rospy.Duration(1.0))
            markerpose = tf2_geometry_msgs.do_transform_pose(mark.pose, transform)
            currentid = 1000000
            currentorder = 1000000
            dontupdate = False
            for landmark in Landmarklist:
                if landmark.id == mark.id:
                    hasbeenseen=True
                    currentid = landmark.id
                    currentorder = landmark.order
                    break
            if not hasbeenseen:
                if len(Landmarklist)<landmarks:
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
                    dontupdate = True
            
            if not dontupdate:
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

                mu_slam = mu_slam + np.matmul(K,z-zpredict)
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
                for landmark in Landmarklist:
                    marktransform = TransformStamped()
                    marktransform.header.frame_id = "map"
                    marktransform.child_frame_id = "arucolandmark" + str(landmark.id)
                    marktransform.header.stamp = msg.header.stamp
                    marktransform.transform.translation.x = mu_slam[landmark.order*2 + 3]
                    marktransform.transform.translation.y = mu_slam[landmark.order*2 + 4]
                    marktransform.transform.translation.z = markerpose.pose.position.z
                    marktransform.transform.rotation.x =  0      #markerpose.pose.orientation.x
                    marktransform.transform.rotation.y =  0      #markerpose.pose.orientation.y
                    marktransform.transform.rotation.z =  0      #markerpose.pose.orientation.z
                    marktransform.transform.rotation.w =  1      #markerpose.pose.orientation.w
                    br.sendTransform(marktransform)
                updaterviz()
                rospy.loginfo("updated")
        else:
            P[0:3,0:3].fill(0)


if __name__ == '__main__':
    rospy.init_node('ekf_slam')
    sub_goal = rospy.Subscriber('/predictedvel', Twist, predict_callback)
    update = rospy.Subscriber('/aruco/markers', ArucoMarkerArray, update_callback)
    tfBuffer = tf2_ros.Buffer(rospy.Duration(12000.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        continue
    
    rospy.spin()