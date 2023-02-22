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
# Denavit-Hartenberg parameters
d = [0.026, 0.09, 0.097, 0.053, 0.08]
print(sum(d))
a = [0.0, 0.0, 0.0, 0.0, 0.0]
alpha = [math.pi/2, 0.0, 0.0, -math.pi/2, 0.0]

def forward_kinematics(q):
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

        

        #T_0E = np.matmul(Trans_di, np.matmul(Rot_thetai, np.matmul(Trans_ai, Rot_alphai)))

        T_0E = np.matmul(T_0E, T_i)
        print(T_0E)
        #T_0E = T_i

        if i < 4:
            z[:,i+1] = T_0E[0:3, 2]
            p[:,i+1] = T_0E[0:3, 3]

    pe = T_0E[0:3,3]

    # Computation of Jacobian
    Jac = np.zeros((6,5))
    for i in range(5):
        Jac[0:3, i] = np.cross(z[:,i], pe-p[:,i])
        Jac[3:, i] = z[:,i]

    return T_0E, Jac


def pose_callback(msg: PoseStamped):
    stamp = msg.header.stamp
    
    try:
        transform = tf_buffer.lookup_transform('base_link', msg.header.frame_id, stamp)
        pose_base = tf2_ros.do_transform_pose(msg.pose, transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Could not get transform")
        return

    # forward kinematics to get current position of end effector
    T_0E, Jac = forward_kinematics(q)
    current_pos = T_0E[0:3,3]
    current_orientation = tf_conversions.transformations.quaternion_from_matrix(T_0E)

    # desired position and orientation
    desired_pos = np.array([pose_base.position.x, pose_base.position.y, pose_base.position.z])
    # desired_orientation = np.array([pose_base.orientation.x, pose_base.orientation.y, pose_base.orientation.z, pose_base.orientation.w])

    # compute error
    pos_error = desired_pos - current_pos
    # orientation_error = tf_conversions.transformations.quaternion_multiply(tf_conversions.transformations.quaternion_inverse(current_orientation), desired_orientation)
    
    # compute velocity
    vel = np.zeros((6,1))
    vel[0:3] = pos_error
    # vel[3:] = orientation_error

    # compute joint velocities
    q_dot = np.matmul(np.linalg.pinv(Jac), vel)

    # publish joint velocities
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    msg.points = [JointTrajectoryPoint(positions=q, velocities=q_dot, time_from_start=rospy.Duration(1.0))]
    pub.publish(msg)

    


def joint_state_callback(msg: JointState):
    global joint_states
    joint_states = msg


def testPick():
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = 0.3
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.05

    
    try:
        map_to_base = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2))
        rotation = np.array((map_to_base.transform.rotation.x, map_to_base.transform.rotation.y, map_to_base.transform.rotation.z, map_to_base.transform.rotation.w))
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(rotation)
        yaw = yaw + math.pi
        quat = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Could not get transform")
        return

    try:
        transform = tf_buffer.lookup_transform("map", pose.header.frame_id, rospy.Time(0), rospy.Duration(2))
        pose_out = tf2_geometry_msgs.do_transform_pose(pose, transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)
        return

    t = TransformStamped()
    t.header = pose.header
    t.header.frame_id = "map"
    t.child_frame_id = "object"

    
    t.transform.translation = pose_out.pose.position
    t.transform.rotation = pose_out.pose.orientation

    tfbroadcaster.sendTransform(t)

    # forward kinematics to get current position of end effector

    q = joint_states.position[0:5]
    T_0E, Jac = forward_kinematics(q)
    current_pos = T_0E[0:3,3]
    current_orientation = tf_conversions.transformations.quaternion_from_matrix(T_0E)

    # desired position and orientation
    desired_pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    # desired_orientation = np.array([pose_base.orientation.x, pose_base.orientation.y, pose_base.orientation.z, pose_base.orientation.w])

    # compute error
    pos_error = desired_pos - current_pos
    # orientation_error = tf_conversions.transformations.quaternion_multiply(tf_conversions.transformations.quaternion_inverse(current_orientation), desired_orientation)
    
    # compute velocity
    vel = np.zeros((6,1))
    vel[0:3] = np.reshape(pos_error, (3,1))
    # vel[3:] = orientation_error

    # compute joint velocities
    q_dot = np.matmul(np.linalg.pinv(Jac), vel)
    q_dot = q_dot.tolist()

    # computer desired joint angles
    q = np.reshape(np.array(q), (5,1))
    q_des = q + q_dot
    q_des = q_des.tolist()
    for i in range(len(q_des)):
        q_des[i] = q_des[i][0]
        q_dot[i] = q_dot[i][0]


    q_des = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0, -1.7802358066666664]
    q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("wait for server")
    client.wait_for_server()
    print("Connected to server")
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    goal.trajectory.points = [JointTrajectoryPoint(positions=q_des, velocities=q_dot, time_from_start=rospy.Duration(1.0))]


    client.send_goal(goal)

    client.wait_for_result()
    print(client.get_result())
    return client.get_result()

    #print(q_des)
    q_des = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0]
    q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]
    # publish joint velocities
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    jtp = JointTrajectoryPoint()
    jtp.positions = q_des
    jtp.velocities = q_dot
    jtp.time_from_start = rospy.Duration(200.0)
    msg.points = [jtp]
    print(msg)
    pub.publish(msg)

    # publish to joint 1
    msg1 = CommandDuration()
    msg1.data = q_des[0]
    msg1.duration = 200.0
    #joint1Pub.publish(msg1)
    



if __name__ == "__main__":
    rospy.init_node('pickup')
    poseSub = rospy.Subscriber('/detection/pose', PoseStamped, pose_callback)
    joint1Pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size=10)
    jointStateSub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(0.1) # 10hz

    q = [0.0, -1.06, -0.81, -1.16, -0.04]
    q = [0.0, 0.0, 0.0, 0.0, 0.0]
    T0E, Jac = forward_kinematics(q)
    print(T0E)
    if(False):
        while not rospy.is_shutdown():
            if joint_states is not None:
                testPick()
                rate.sleep()