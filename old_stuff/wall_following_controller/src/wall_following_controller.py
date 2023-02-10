#!/usr/bin/env python3
import rospy
from robp_msgs.msg import Encoders
from robp_msgs.msg import DutyCycles
from geometry_msgs.msg import Twist
from robp_boot_camp_msgs.msg import ADConverter
import math


class Wallfollower(object):
    def __init__(self):
        
        rospy.Subscriber("/kobuki/adc", ADConverter, self.callback)

    def callback(self,data):
            self.adc = ADConverter()
            
            alpha = 20 # alpha should be positive, tune it.

            self.adc.ch1 = data.ch1
            self.adc.ch2 = data.ch2

            self.d1 = 1.114*math.exp(-0.004*self.adc.ch1)
            self.d2 = 1.114*math.exp(-0.004*self.adc.ch2)

            angular_vel = alpha * (self.d1 - self.d2)
            linear_vel = 10

            self.publish(angular_vel, linear_vel)


    def publish(self, angular_vel, linear_vel):
        "\n"
        print("Publishing to the motor_controller/twist topic...")
        self.pub = rospy.Publisher('/motor_controller/twist', Twist, queue_size=10) #motor_controller/twist
        self.rate = rospy.Rate(10) # 10hz
        #self.angular_vel = self.angular_vel + 0.1
        self.velo = Twist()

        self.velo.linear.x = linear_vel
        self.velo.angular.z = angular_vel
        
        rospy.loginfo(self.velo)

        self.pub.publish(self.velo)
        self.rate.sleep()
        

if __name__ == '__main__':
    rospy.init_node("wall_following_controller", anonymous=True)
    try:
        Wallfollower()
        
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped