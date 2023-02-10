#!/usr/bin/env python3
import rospy
from robp_msgs.msg import Encoders
from robp_msgs.msg import DutyCycles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class Listener(object):
    def __init__(self):

        self.encoder_sub = rospy.Subscriber("/motor/encoders", Encoders, self.callback1)
        self.theta_sub = rospy.Subscriber("/odom", Odometry, self.callback2)
        self.velocity_sub = rospy.Subscriber("/motor_controller/twist", Twist, self.callback3)

        print("Initializing the instance!")

    def callback1(self, data):
        self.encoders = Encoders()
        self.encoders.delta_encoder_left = data.delta_encoder_left
        self.encoders.delta_encoder_right = data.delta_encoder_right

    
    def callback2(self,data):
        self.odometry = Odometry()
        self.odometry.pose.pose.orientation.w = data.pose.pose.orientation.w

    def callback3(self, data):
        self.twist = Twist()
        self.twist.linear.x = data.linear.x
        self.twist.angular.z = data.angular.z
        self.desired_w = self.twist.angular.z

        b = 0.23
        r = 0.0352
        f  = 10
        ticks_per_rev = 360

        self.estimated_vw1 = (2*math.pi*r*f*self.encoders.delta_encoder_left)/ticks_per_rev
        self.estimated_vw2 = (2*math.pi*r*f*self.encoders.delta_encoder_right)/ticks_per_rev
        #print('Callback 3 executed!')
        
        
        x_dot = (math.cos(self.odometry.pose.pose.orientation.w)) * self.twist.linear.x
        z_dot = self.twist.angular.z
        self.desired_v1 = x_dot-b*(z_dot)
        self.desired_v2 = b*(z_dot)+(x_dot)
        self.error_v1 = self.desired_v1 - self.estimated_vw1
        self.error_v2 = self.desired_v2 - self.estimated_vw2

        Kp1 = 0.15 # alpha should be positive, tune it. 
        Kp2 = 0.132
        p_controller_v1 = self.error_v1*Kp1
        p_controller_v2 = self.error_v2*Kp2
        
        print(p_controller_v1)

        self.publish(p_controller_v1, p_controller_v2)
        

        #error = self.desired_w - self.w_estimate # if estimated is bigger than desired, then the robot rill turn right.
        

    

        

    def publish(self,p_controller_v1,p_controller_v2):
        "\n"
        #print("Here we publish to the dutycyle topic")
        self.vel_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        self.message = DutyCycles()
        
        self.message.duty_cycle_left = p_controller_v1
        self.message.duty_cycle_right = p_controller_v2

        self.vel_pub.publish(self.message)
        rate.sleep()
        

if __name__ == '__main__':
    rospy.init_node("cartesian_controller", anonymous=True)
    try:
        listener = Listener()
        
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.