#! /usr/bin/env python3
import rospy
import armpi_fpv.bus_servo_control as bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from math import cos, sin, acos, atan2, pi
import math
from std_msgs.msg import Float64
import numpy as np

# class arm_to_goal(object):
#     def __init__(self):
#         print("initiated!!!!!!!!!!!!!!!!!!!!!!")
def scara_IK(point):
    print("made it into scara_IK")
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]


    a0 = 0.07
    a1 = 0.3
    a2 = 0.35
    
    rad = math.sqrt((x-a0)**2 + y**2)
    
    cos_th2 = ((rad**2-a1**2-a2**2)/(2*a1*a2))
    sin_th2 = math.sqrt(1-cos_th2**2)
    th2 = math.atan2(sin_th2,cos_th2)
    s1 = ((a1 + a2*cos_th2)*y - a2*sin_th2*(x-a0))/(rad**2)
    c1 = ((a1 + a2*cos_th2)*(x-a0) + a2*sin_th2*y)/(rad**2)
    th1 = math.atan2(s1,c1)

    q[0] = th1
    q[1] = th2
    q[2] = z
    pub_join1.publish(q)
        # return q

if __name__ == "__main__":
    rospy.init_node("robot_arm")
    rospy.loginfo("Starting robot arm node")
    point =[0,0,0]

    # tfBuffer = tf2_ros.Buffer()
    # tflistener = tf2_ros.TransformListener(tfBuffer)

    pub_join1 = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    try:
        scara_IK(point)
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
# def ForwardKinematicsAndJacobian(q, alpha, d, a):

#     # transformation matrix 0TE for forward kinematics
#     # and parameters for Jacobian computation

#     T_0E = np.identity((4))
#     z = np.zeros((3,7))
#     p = np.zeros((3,7))
#     pe = np.zeros((3,1))

#     z[:,0] = np.transpose(np.array([0, 0, 1]))

#     for i in range(7):
#         Rot_thetai = np.array([[cos(q[i]), -sin(q[i]), 0, 0], [sin(q[i]), cos(q[i]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#         Rot_alphai = np.array([[1, 0, 0, 0], [0, cos(alpha[i]), -sin(alpha[i]), 0], [0, sin(alpha[i]), cos(alpha[i]), 0], [0, 0, 0, 1]])
#         Trans_di = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d[i]], [0, 0, 0, 1]])
#         Trans_ai = np.array([[1, 0, 0, a[i]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

#         T_i = np.matmul(Trans_di, Rot_thetai)
#         T_i = np.matmul(T_i, Trans_ai)
#         T_i = np.matmul(T_i, Rot_alphai)

#         T_0E = np.matmul(T_0E, T_i)

#         if i < 6:
#             z[:,i+1] = T_0E[0:3, 2]
#             p[:,i+1] = T_0E[0:3, 3]

#     pe = T_0E[0:3,3]
#     # Computation of Jacobian
#     Jac = np.zeros((6,7))
#     for i in range(7):
#         Jac[0:3, i] = np.cross(z[:,i], pe-p[:,i])
#         Jac[3:, i] = z[:,i]

#     return T_0E, Jac

