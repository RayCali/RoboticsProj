#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from sensor_msgs.msg import Imu
x=0.0
y=0.0
yaw = 0.0

def imu_callback(msg: Imu):
    global x,y,yaw
    rospy.loginfo('New encoder received:\n%s', msg)

    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"

 
    r= 0.04921
    dt=1/90.9
    b=0.3
    wx= msg.linear_acceleration.x #linear acceleration in x axis this is the vw_1 and vw_2
    vz = msg.angular_velocity.z #rotation around z axis
    v = wx
    w= (vz)/(b)
    diffx=v*dt*math.cos(yaw)
    diffy=v*dt*math.sin(yaw)
    difftheta=w*dt
    x=x+diffx
    y=y+diffy
    yaw = yaw + difftheta
    t.transform.translation.x = x
    t.transform.translation.y = y
    
    t.header.stamp=msg.header.stamp
    t.transform.translation.x = x
    t.transform.translation.y = y
    rospy.loginfo("x= %f" % x)
    rospy.loginfo("y= %f" % y)
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


rospy.init_node('odometry_imu')
# sub_goal = rospy.Subscriber('/motor/encoders', Encoders, encoder_callback)
sub_imu = rospy.Subscriber('/imu', Imu, imu_callback)
if __name__ == '__main__':
    rospy.spin()