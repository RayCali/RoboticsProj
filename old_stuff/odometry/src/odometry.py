#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math

x = 0.0
y = 0.0
theta = 0.0
def encoder_callback(msg):
    rospy.loginfo('New encoder received:\n%s', msg)
    global x
    global y
    global theta

    br = tf2_ros.TransformBroadcaster()
    
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
 
    # TODO: Fill in
    ticks_per_revolution = 3072
    r = 0.04921
    B = 0.3
    K = 0.002
   
    delta_encoder_left = msg.delta_encoder_left
    delta_encoder_right = msg.delta_encoder_right
    D = (r/2)*(K*delta_encoder_right + K*delta_encoder_left)
    delta_theta = (r/B)*(K*delta_encoder_right - K*delta_encoder_left)
    
    
    x = x + D * math.cos(theta)
    y = y + D * math.sin(theta)
    theta = theta + delta_theta
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    yaw = theta
    
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)

    #t.header.frame_id = msg.header.frame_id
    
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    print(t)
    br.sendTransform(t)

rospy.init_node('odometry')

sub_goal = rospy.Subscriber('/motor/encoders', Encoders, encoder_callback)

if __name__ == '__main__':
    rospy.spin()