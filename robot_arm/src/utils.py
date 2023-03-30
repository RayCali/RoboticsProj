#!/usr/bin/env python3

import numpy as np
from math import sin, cos
import math

# home position
q_home = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0]
gripper_open = -1.7802358066666664
gripper_closed = 0.0


# Denavit-Hartenberg parameters
d = [0.015, 0.0, 0.0, 0.0, 0.0]
a = [0.0, 0.1, 0.096, 0.055, 0.085]
alpha = [math.pi/2, 0.0, 0.0, -math.pi/2, 0.0]

def forward_kinematics(q):
    """
    Forward kinematics for robot arm.
    input:
            joint angles [q1, q2, q3, q4, q5]
    output:
            transformation matrix T_0E
    """
    q[1] += math.pi/2
    # transformation matrix 0TE for forward kinematics
    T_0E = np.identity((4))

    for i in range(5):
        Rot_thetai = np.array([[cos(q[i]), -sin(q[i]), 0, 0], [sin(q[i]), cos(q[i]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Rot_alphai = np.array([[1, 0, 0, 0], [0, cos(alpha[i]), -sin(alpha[i]), 0], [0, sin(alpha[i]), cos(alpha[i]), 0], [0, 0, 0, 1]])
        Trans_di = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d[i]], [0, 0, 0, 1]])
        Trans_ai = np.array([[1, 0, 0, a[i]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        T_i = np.matmul(Trans_di, Rot_thetai)
        T_i = np.matmul(T_i, Trans_ai)
        T_i = np.matmul(T_i, Rot_alphai)

        T_0E = np.matmul(T_0E, T_i)

    return T_0E

def analyticalIK_lock4(position):
    """
    Inverse kinematics for robot arm.
    input:
            pose (x,y,z) in arm_base frame
    output:
            joint angles [q1, q2, q3, q4, q5]
    """

    # lock q4 and q5
    q4 = -math.pi/2
    q5 = 0.0

    x = position[0]
    y = position[1]
    z = position[2]

    # rotate arm towards object
    q1 = math.atan2(y, x)

    # compute q3 and q2
    l0 = d[0]
    l1 = a[1]
    l2 = a[2]
    l3 = a[3]
    l4 = a[4]

    z = z-l0 # subtract offset

    l2_eff = math.sqrt(l2**2 + (l3+l4)**2 - 2*l2*(l3+l4)*math.cos(math.pi-abs(q4)))
    # print(l2_eff)

    # print((x**2 + z**2 - (l1**2 + l2_eff**2))/(2*l1*l2_eff))
    q3_eff = -math.acos((x**2 + z**2 - (l1**2 + l2_eff**2))/(2*l1*l2_eff))
    q2 = -math.atan2(x, z) - math.atan2(l2_eff*math.sin(q3_eff), l1 + l2_eff*math.cos(q3_eff))

    angle_offset = math.acos((l2_eff**2+l2**2-(l3+l4)**2)/(2*l2_eff*l2))
    # print("inner angle: ",angle_offset)
    q3 = q3_eff + angle_offset
    # print("eff:", [q1, q2, q3_eff, q4, q5])
    q = [q1, q2, q3, q4, q5]

    x_ = -l1*math.sin(q2) - l2_eff*math.sin(q2+q3)
    z_ = l1*math.cos(q2) + l2_eff*math.cos(q2+q3) + l0

    # print(q)
    # print([x_,z_])
    
    return q