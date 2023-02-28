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
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

joint_states = None

# home position
home = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0]

# Denavit-Hartenberg parameters
d = [0.02, 0.0, 0.0, 0.0, 0.0]
a = [0.0, 0.09, 0.097, 0.053, 0.08]
alpha = [math.pi/2, 0.0, 0.0, -math.pi/2, 0.0]

def forward_kinematics(q):
    # forward kinematics for robot arm
    # input: joint angles q1, q2, q3, q4, q5
    # output: transformation matrix T_0E and Jacobian
    q[1] += math.pi/2
    # transformation matrix 0TE for forward kinematics
    T_0E = np.identity((4))
    # and parameters for Jacobian computation
    z = np.zeros((3,5))
    p = np.zeros((3,5))
    pe = np.zeros((3,1))

    z[:,0] = np.transpose(np.array([0, 0, 1]))
    for i in range(5):
        Rot_thetai = np.array([[cos(q[i]), -sin(q[i]), 0, 0], [sin(q[i]), cos(q[i]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Rot_alphai = np.array([[1, 0, 0, 0], [0, cos(alpha[i]), -sin(alpha[i]), 0], [0, sin(alpha[i]), cos(alpha[i]), 0], [0, 0, 0, 1]])
        Trans_di = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d[i]], [0, 0, 0, 1]])
        Trans_ai = np.array([[1, 0, 0, a[i]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        T_i = np.matmul(Trans_di, Rot_thetai)
        T_i = np.matmul(T_i, Trans_ai)
        T_i = np.matmul(T_i, Rot_alphai)

        T_0E = np.matmul(T_0E, T_i)

    #     if i < 4:
    #         z[:,i+1] = T_0E[0:3, 2]
    #         p[:,i+1] = T_0E[0:3, 3]

    # pe = T_0E[0:3,3]

    # # Computation of Jacobian
    # Jac = np.zeros((6,5))
    # for i in range(5):
    #     Jac[0:3, i] = np.cross(z[:,i], pe-p[:,i])
    #     Jac[3:, i] = z[:,i]

    return T_0E

def analyticalIK(position):
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
    l1 = 0.09
    l2 = 0.097
    l3 = 0.053
    l4 = 0.08

    l2_eff = math.sqrt(l2**2 + (l3+l4)**2)

    print((x**2 + z**2 - (l1**2 + l2_eff**2))/(2*l1*l2_eff))
    q3 = math.acos((x**2 + z**2 - (l1**2 + l2_eff**2))/(2*l1*l2_eff))
    q2 = -math.atan2(x, z) - math.atan2(l2_eff*math.sin(q3), l1 + l2_eff*math.cos(q3))

    q = [q1, q2, q3, q4, q5]
    
    return q

def pose_callback(msg: PoseStamped):
    stamp = msg.header.stamp

    try:
        tf_buffer.lookup_transform('arm_base', msg.header.frame_id, stamp, timeout=rospy.Duration(4.0))
        pose_base = tf_buffer.transform(msg, 'arm_base')
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Could not get transform")
        return
    
    # forward kinematics to get current position of end effector
    # T_0E = forward_kinematics(q)
    # current_pos = T_0E[0:3,3]

    # go to home position
    q_home = home

    # go to hover position
    pos_hover = [pose_base.pose.position.x, pose_base.pose.position.y, 0.02]
    q_hover = analyticalIK(pos_hover)

    # go to desired position
    pos_pick = [pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z]
    q_pick = analyticalIK(pos_pick)

    #positions = [q_home, q_hover, q_pick]

    q0 = [0.0, 0.0, 0.0, 0.0, 0.0]

    print("Waiting for trajectory controller...")
    trajectory_client.wait_for_server()
    print("Connected to server.")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    goal.trajectory.points = [JointTrajectoryPoint(positions=q_home, velocities=q0, time_from_start=rospy.Duration(1.0)),
                              JointTrajectoryPoint(positions=q0, velocities=q0, time_from_start=rospy.Duration(10.0))]
    # goal.trajectory.points = [JointTrajectoryPoint(positions=q_home, velocities=[], time_from_start=rospy.Duration(5.0)),
    #                           JointTrajectoryPoint(positions=q_hover, velocities=[], time_from_start=rospy.Duration(10.0)),
    #                           JointTrajectoryPoint(positions=q_pick, velocities=[], time_from_start=rospy.Duration(15.0))]
    print("Sending goal...")
    trajectory_client.send_goal(goal)
    
    trajectory_client.wait_for_result()
    print("Finished")

    print(trajectory_client.get_result())    


def joint_state_callback(msg: JointState):
    global joint_states
    joint_states = msg


if __name__ == "__main__":
    rospy.init_node('pickup')
    #poseSub = rospy.Subscriber('/detection/pose', PoseStamped, pose_callback)
    joint1Pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size=10)
    jointStateSub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    trajectory_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(1000) # 10hz

    test_pose = PoseStamped()
    test_pose.header.frame_id = "base_link"
    test_pose.header.stamp = rospy.Time.now()
    test_pose.pose.position.x = 0.05
    test_pose.pose.position.y = 0.02
    test_pose.pose.position.z = -0.1
    test_pose.pose.orientation.x = 0.0
    test_pose.pose.orientation.y = 0.0
    test_pose.pose.orientation.z = 0.0
    test_pose.pose.orientation.w = 1.0


    joint1_command = CommandDuration()
    joint1_command.duration = 200.0
    joint1_command.data = 1.0

    print("Publishing")
    joint1Pub.publish(joint1_command)
    rospy.spin()
    exit()

    q_des = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0, -1.7802358066666664]
    q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("wait for server")
    client.wait_for_server()
    print("Connected to server")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    goal.trajectory.points = [JointTrajectoryPoint(positions=q_des, velocities=q_dot, time_from_start=rospy.Duration(1.0))]

    print("Sending goal")
    client.send_goal(goal)

    client.wait_for_result()
    print(client.get_result())
    #return client.get_result()
    exit()

    while not rospy.is_shutdown():
        if joint_states is not None:
            pose_callback(test_pose)
            rate.sleep()