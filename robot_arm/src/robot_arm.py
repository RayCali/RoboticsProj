#! /usr/bin/env python3
# encoding:utf-8
import rospy
# from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from math import cos, sin, acos, atan2, pi
import math
from std_msgs.msg import Float64
import numpy as np

import time
import numpy as np
from math import sqrt
if __name__ == "__main__":
    import inverse_kinematics
else:
    from . import inverse_kinematics

#Robotic arm moves based on the angle calculated by the inverse kinematics.
ik = inverse_kinematics.IK()

class ArmIK:
    servo3Range = (0, 1000, 0, 240.0) #Pulse width, angle 
    servo4Range = (0, 1000, 0, 240.0)
    servo5Range = (0, 1000, 0, 240.0)
    servo6Range = (0, 1000, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # adapt to the different servos
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        #Convert the angle calculated by inverse kinematics into the corresponding pulse width of servo.
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        servo4 = int(round(-theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 + theta5 * self.servo5Param))
        servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + theta6)) * self.servo6Param)
        
        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}
    
    def setPitchRanges(self, coordinate_data, alpha, alpha1, alpha2, d = 0.01):
        #According to the given coordinate_data, the pitch angle alpha and the range of the pitch angle alpha1, alpha2, automatically search for a solution for the pitch angle.
        #If there is no solution, return False. Otherwise, return the servo angle and the pitch angle.
        #The given Coordinate unit m, passed in as a tuple, for example (0, 0.5, 0.1)
        #alpha1 is the given pitch angle and the unit is degree.
        #alpha1 and alpha2 is the range of pitch angle
        x, y, z = coordinate_data
        a_range = abs(int(abs(alpha1 - alpha2)/d)) + 1
        for i in range(a_range):
            if i % 2:
                alpha_ = alpha + (i + 1)/2*d
            else:                
                alpha_ = alpha - i/2*d
                if alpha_ < alpha1:
                    alpha_ = alpha2 - i/2*d
            result = ik.getRotationAngle((x, y, z), alpha_)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                return result, servos, alpha_
        
        return False

if __name__ == "__main__":
    import rospy
    import armpi_fpv.bus_servo_control as bus_servo_control
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur
    
    # Initialization node
    rospy.init_node('ik_test', log_level=rospy.DEBUG)
    # Servo publish
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)
    
    AK = ArmIK()
    print(ik.getLinkLength())
    target = AK.setPitchRanges((0.0, 0.12, 0.08), -145, -180, 0)
    
    if target:
        print(target)
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((1, 450), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))

    
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

