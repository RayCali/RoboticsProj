#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
x=0
y=0
yaw=0
def encoder_callback(msg):
    global x,y,yaw
    rospy.loginfo('New encoder received:\n%s', msg)

    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
 
    
    # 2*math.pi/3072 = radians per tick
    r= 0.04921
    dt=1/20
    b=0.3
    vw_1 = msg.delta_encoder_left*2*math.pi*20*r/3072
    vw_2 = msg.delta_encoder_right*2*math.pi*20*r/3072
    v = (vw_1+vw_2)/2
    w= (vw_2-vw_1)/(b)
    diffx=v*dt*math.cos(yaw)
    diffy=v*dt*math.sin(yaw)
    difftheta=w*dt
    x=x+diffx
    y=y+diffy
    yaw = yaw + difftheta #
    t.transform.translation.x = x
    t.transform.translation.y = y
    
    #K=2*math.pi*20/3072
    #D=(r/2)*(K*msg.delta_encoder_left + K*msg.delta_encoder_right)
    #DTheta=(r/b)*(K*msg.delta_encoder_right - K*msg.delta_encoder_left)
    #x=x+D*math.cos(yaw)
    #y=y+D*math.sin(yaw)
    t.header.stamp=rospy.Time.now()
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

rospy.init_node('odometry')
sub_goal = rospy.Subscriber('/motor/encoders', Encoders, encoder_callback)

if __name__ == '__main__':
    rospy.spin()