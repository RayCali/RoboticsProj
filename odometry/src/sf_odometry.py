#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from sensor_msgs.msg import Imu
import numpy as np
x=0.0
y=0.0
yaw = 0.0
imu_lin = []
imu_ang = []
mu = np.zeros(2)
sigma = np.zeros((2,2))

def encoder_callback(msg):
    global x,y,yaw,sigma,mu,imu_lin,imu_ang

    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
 
    
    # 2*math.pi/3072 = radians per tick
    
    #Predict
    r= 0.04921
    dt=1/20
    b=0.3
    vw_1 = msg.delta_encoder_left*2*math.pi*20*r/3072
    vw_2 = msg.delta_encoder_right*2*math.pi*20*r/3072
    v = (vw_1+vw_2)/2
    w= (vw_2-vw_1)/(b)
    mu = np.array([v,w])
    G = np.identity(2)
    sigma = G @ sigma @ G.T + np.array([[0.3,0],[0,0.3]])
    
    
    #Update
    if len(imu_lin) != 0:
        vupdate = dt*np.sum(imu_lin)/len(imu_lin)
        wupdate = np.sum(imu_ang)/len(imu_ang)
        z = np.array([vupdate,wupdate])
        H = np.identity(2)
        K = sigma @ H.T @ np.linalg.inv(H @ sigma @ H.T + np.array([[0.1,0],[0,0.1]]))
        mu = mu + K @ (z - mu)
        sigma = (np.identity(2) - K @ H) @ sigma
        imu_lin = []
        imu_ang = []
        rospy.loginfo("updated")
    
    
    v = mu[0]
    w = mu[1]
    diffx=v*dt*math.cos(yaw)
    diffy=v*dt*math.sin(yaw)
    difftheta=w*dt
    x=x+diffx
    y=y+diffy
    yaw = yaw + difftheta #
    

    t.transform.translation.x = x
    t.transform.translation.y = y
    
    t.header.stamp=msg.header.stamp
    t.transform.translation.x = x
    t.transform.translation.y = y
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def imu_callback(msg:Imu):
    global x,y,yaw, imu_lin, imu_ang
    imu_lin.append(msg.linear_acceleration.x)
    imu_ang.append(msg.angular_velocity.z)
    



rospy.init_node('sf_odometry')
sub_goal = rospy.Subscriber('/motor/encoders', Encoders, encoder_callback)
sub_imu = rospy.Subscriber('/imu/data', Imu, imu_callback)
if __name__ == '__main__':
    rospy.spin()