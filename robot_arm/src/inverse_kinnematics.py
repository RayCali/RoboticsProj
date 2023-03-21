#!/usr/bin/env python3
import rospy
import math
import numpy as np
from math import sin, cos, pi , atan2

# Create transformation matrix between two links 
# according to Modified DH convention with given parameters  
def createMatrix(alpha, a, q, d):
    mat =  [[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]]
    
    print(mat)

    return mat

pg_0  = [px, py, pz] # End effector position.
p_quat = [qx, qy, qz, qw] # End effector orientation as a quaternion.

# R0_g = end-effector(gripper) rotation transformation(4X4)
R0_g = tf.transformations.quaternion_matrix(p_quat)
D0_g = tf.transformations.translation_matrix(pg_0)

T_total = R0_g*D0_g
# rwc_0 = wrist-center position w.r.t. base_link(O_0)
rwc_0 = pg_0 - (d_g*(R0_g*z_g))
theta1 = atan2(rwc_0[1], rwc_0[0])

# position of O2 w.r.t. base_link in zero configuration, i.e. when q1 = 0
pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])
# Rotate pO2_0 w.r.t. Z by theta1
pO2_0 = rot_z(theta1)* pO2_0


if __name__ == "__main__":
    rospy.init_node("inverse_kinnematics")

    try:
        createMatrix(0, 0, 0, 0)
        
    except rospy.ROSInterruptException:
        pass

    rospy.spin()