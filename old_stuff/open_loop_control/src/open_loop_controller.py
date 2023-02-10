#!/usr/bin/env python3
# license removed for brevity
import rospy
from robp_msgs.msg import DutyCycles

def publisher():
    pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
    rospy.init_node('open_loop_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = DutyCycles()
        message.duty_cycle_left = 0.5
        message.duty_cycle_right = 0.5
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass