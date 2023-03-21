#!/usr/bin/python3
from math import pi
import rospy
from robp_msgs.msg import DutyCycles
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from robp_msgs.msg import Encoders 
from sensor_msgs.msg import Imu
IMUdata=None
desiredTwist=None
dt= 1/90.9 #90.9 is the rate of publishing fr imu
int_error1 = 0
int_error2 = 0 
alpha1=0.02078
alpha2=0.02
beta1=0.0000022
beta2=0.000002

def callbackTwist(data):
    global desiredTwist
    # rospy.loginfo("got twist")
    desiredTwist=data

def callbackEncoder(data):
    global Efeedback
    # rospy.loginfo("nu")
    Efeedback=data

def listener():
    rospy.Subscriber('/cmd_vel',Twist, callbackTwist)
    rospy.Subscriber('/motor/encoders',Encoders, callbackEncoder)
    

def PI():
    global desiredTwist
    global Efeedback
    global int_error2
    global int_error1
    global alpha1
    global alpha2
    global beta1
    global beta2
    pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    mesg = DutyCycles()
    rospy.sleep(5)
    int_max=10000
   
    while not rospy.is_shutdown():
        #rospy.loginfo("im in the mainframe")
        # rospy.loginfo("lesgo")
        if(Efeedback is None or desiredTwist is None):
            continue
        r=0.04921
        b=0.3
        dt=Efeedback.delta_time_left
        w_w1=2*pi*Efeedback.delta_encoder_left/3072
        w_w2=2*pi*Efeedback.delta_encoder_right/3072
        #rospy.loginfo("Wheel 1= %f" % w_w1)
        #rospy.loginfo("Wheel 2= %f" % w_w2)
        #v_w2d=(2*desiredTwist.linear.x-v_w1d)
        #w*2*b = (2*desiredTwist.linear.x-v_w1d)-v_w1d
        v_w1d=(2*b*desiredTwist.angular.z-2*desiredTwist.linear.x)/(-2)
        v_w2d=(2*desiredTwist.linear.x-v_w1d)
        w_w1d = v_w1d/r
        w_w2d = v_w2d/r
        error1 = w_w1d-w_w1
        int_error1 = int_error1 + error1 * dt #10000
        #rospy.loginfo("int_error 1= %f" % int_error1)
        if int_error1>int_max:
            int_error1=0
        if int_error2>int_max:
            int_error2=0
        pwm1 = alpha1 * error1 + beta1 * int_error1
        error2 = w_w2d - w_w2
        int_error2 = int_error2 + error2 * dt
        #rospy.loginfo("int_error 2= %f" % int_error2)
        pwm2 = alpha2 * error2 + beta2 * int_error2
        mesg.duty_cycle_left=pwm1
        mesg.duty_cycle_right=pwm2
        # rospy.loginfo("waiting for data")

        pub.publish(mesg)
        rate.sleep()
        if desiredTwist.linear.x==0 and desiredTwist.angular.z==0:
            mesg.duty_cycle_left=0.0
            mesg.duty_cycle_right=0.0
            int_error1=0
            int_error2=0

            pub.publish(mesg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('cartesian_controller', anonymous=True)
    try:
        listener()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
        pass
    try:
        PI()
    except rospy.ROSInterruptException:
        rospy.loginfo(e)
        pass
    rospy.loginfo("spinning")
    rospy.spin()