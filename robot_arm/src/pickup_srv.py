#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from robp_msgs.msg import Encoders
from std_msgs.msg import Float64
from hiwonder_servo_msgs.msg import CommandDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from math import sin, cos
import math
import actionlib
from actionlib import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


joint_states = None
pose_stamped = None

# home position
home = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0]
gripper_open = -1.7802358066666664
gripper_closed = -0.3


# Denavit-Hartenberg parameters
d = [0.015, 0.0, 0.0, 0.0, 0.0]
a = [0.0, 0.1, 0.096, 0.055, 0.085]
alpha = [math.pi/2, 0.0, 0.0, -math.pi/2, 0.0]

def forward_kinematics(q):
    # forward kinematics for robot arm
    # input: joint angles q1, q2, q3, q4, q5
    # output: transformation matrix T_0E and Jacobian
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
    # inverse kinematics for robot arm
    # input: pose (x,y,z) in base_link frame
    # output: joint angles q1, q2, q3, q4, q5

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
    print(l2_eff)

    print((x**2 + z**2 - (l1**2 + l2_eff**2))/(2*l1*l2_eff))
    q3_eff = -math.acos((x**2 + z**2 - (l1**2 + l2_eff**2))/(2*l1*l2_eff))
    q2 = -math.atan2(x, z) - math.atan2(l2_eff*math.sin(q3_eff), l1 + l2_eff*math.cos(q3_eff))

    angle_offset = math.acos((l2_eff**2+l2**2-(l3+l4)**2)/(2*l2_eff*l2))
    print("inner angle: ",angle_offset)
    q3 = q3_eff + angle_offset
    print("eff:", [q1, q2, q3_eff, q4, q5])
    q = [q1, q2, q3, q4, q5]

    x_ = -l1*math.sin(q2) - l2_eff*math.sin(q2+q3)
    z_ = l1*math.cos(q2) + l2_eff*math.cos(q2+q3) + l0

    print(q)
    print([x_,z_])
    
    return q

def pose_callback(msg: PoseStamped):
    global pose_stamped
    pose_stamped = msg

def joint_state_callback(msg: JointState):
    global joint_states
    joint_states = msg

def handle_pickup_req(req: TriggerRequest):

    # transform pose given in base_link to arm_base
    stamp = pose_stamped.header.stamp

    try:
        tf_buffer.lookup_transform('arm_base', pose_stamped.header.frame_id, stamp, timeout=rospy.Duration(4.0))
        pose_base = tf_buffer.transform(pose_stamped, 'arm_base')
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Could not get transform")
        return

    print("POSE_BASE: ", pose_base)
    # go to hover position
    pos_hover = [pose_base.pose.position.x - 0.02, pose_base.pose.position.y, 0.0]
    q_hover = analyticalIK_lock4(pos_hover)

    # go to desired position
    pos_pick = [pose_base.pose.position.x + 0.02, pose_base.pose.position.y, pose_base.pose.position.z]
    q_pick = analyticalIK_lock4(pos_pick)

    #positions = [q_home, q_hover, q_pick]

    #q_des = [0.0, 0.0, 0.0, 0.0, 0.0]
    q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]
    

    print("wait for server")
    client.wait_for_server()
    print("Connected to server")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5)),
                              JointTrajectoryPoint(positions=q_pick, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
    
    # goal.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]

    print("Sending goal")
    client.send_goal(goal)

    client.wait_for_result()
    
    if client.get_state() == GoalStatus.SUCCEEDED and True:
        # pick up object
        closeGripper = CommandDuration(duration=200.0)
        closeGripper.data = gripper_closed
        gripperPub.publish(closeGripper)

        rospy.sleep(1.0)

        # go to hover position
        goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5))]
        client.send_goal(goal)
        client.wait_for_result()

        # place object
        pos_hover[1] -= 0.05
        q_hover = analyticalIK_lock4(pos_hover)
        pos_pick[1] -= 0.05
        q_pick = analyticalIK_lock4(pos_pick)
        goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5)),
                                  JointTrajectoryPoint(positions=q_pick, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
        client.send_goal(goal)
        client.wait_for_result()

        # open gripper
        openGripper = CommandDuration(duration=200.0)
        openGripper.data = gripper_open
        gripperPub.publish(openGripper)

        rospy.sleep(1.0)

        # go to home position
        goal.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
        client.send_goal(goal)
        client.wait_for_result()

    return TriggerResponse(True, "Success")


if __name__ == "__main__":
    rospy.init_node("pickup_server")
    
    rospy.Service("/pickup", Trigger, handle_pickup_req)

    poseSub = rospy.Subscriber("/detection/pose", PoseStamped, pose_callback)
    jointStateSub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    gripperPub = rospy.Publisher('/r_joint_controller/command_duration', CommandDuration, queue_size=1)
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz

    rospy.spin()