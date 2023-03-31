#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
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
from robot_arm.srv import Pick, PickRequest, PickResponse

from utils import *


joint_states = None
# pose_stamped = None


# def pose_callback(msg: PoseStamped):
#     global pose_stamped
#     pose_stamped = msg

def joint_state_callback(msg: JointState):
    global joint_states
    joint_states = msg

def handle_pickup_req(req: PickRequest):

    if joint_states.position[-1] == gripper_open:

        # transform pose given in base_link to arm_base
        pose_stamped = req.pose
        stamp = pose_stamped.header.stamp

        try:
            tf_buffer.lookup_transform('arm_base', pose_stamped.header.frame_id, stamp, timeout=rospy.Duration(4.0))
            pose_base = tf_buffer.transform(pose_stamped, 'arm_base')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Could not get transform")
            return

        print("POSE_BASE: ", pose_base)
        if pose_base.pose.position.x < 0.2:
            return PickResponse(False, "Object too close to robot")
        # go to hover position
        pos_hover = [pose_base.pose.position.x - 0.02, pose_base.pose.position.y, 0.0]
        q_hover = analyticalIK_lock4(pos_hover)

        # go to desired position
        pos_pick = [pose_base.pose.position.x + 0.02, pose_base.pose.position.y, pose_base.pose.position.z]
        q_pick = analyticalIK_lock4(pos_pick)

        q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]

        print("wait for server")
        trajectory_client.wait_for_server()
        print("Connected to server")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5)),
                                JointTrajectoryPoint(positions=q_pick, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
        
        # goal.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]

        print("Sending goal")
        trajectory_client.send_goal(goal)

        trajectory_client.wait_for_result()
        
        if trajectory_client.get_state() == GoalStatus.SUCCEEDED and True:
            # pick up object
            closeGripper = CommandDuration(duration=200.0)
            closeGripper.data = gripper_closed
            gripperPub.publish(closeGripper)

            rospy.sleep(1.0)

            # go to hover position
            goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5))]
            trajectory_client.send_goal(goal)
            trajectory_client.wait_for_result()

            # go to home position
            goal.trajectory.points = [JointTrajectoryPoint(positions=q_home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
            trajectory_client.send_goal(goal)
            trajectory_client.wait_for_result()

    else:
        return PickResponse(False, "Gripper is already closed, cannot pick up object.")

    return PickResponse(True, "Success")


if __name__ == "__main__":
    rospy.init_node("pickup_server")
    
    rospy.Service("/pickup", Pick, handle_pickup_req)

    poseSub = rospy.Subscriber("/detection/pose", PoseStamped, pose_callback)
    jointStateSub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    gripperPub = rospy.Publisher('/r_joint_controller/command_duration', CommandDuration, queue_size=1)
    trajectory_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz

    rospy.spin()