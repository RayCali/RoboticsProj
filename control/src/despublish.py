#!/usr/bin/python3
import rospy
from robp_msgs.msg import DutyCycles
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def talker():
    
    pub = rospy.Publisher('/motor_controller/twist', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    mesg = Twist()
    while not rospy.is_shutdown():
        mesg.linear.x=0
        mesg.angular.z=1
        pub.publish(mesg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('despublish', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()