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
#from robot_arm.srv import Place, PlaceResponse, PlaceRequest
from msg_srv_pkg.srv import Place, PlaceResponse, PlaceRequest

from utils import *


joint_states = None
pose_stamped = None

dropoff = PoseStamped()
dropoff.header.frame_id = "arm_base"
dropoff.pose.position.x = 0.2
dropoff.pose.position.y = 0.0
dropoff.pose.position.z = -0.05
dropoff.pose.orientation.x = 0.0
dropoff.pose.orientation.y = 0.0
dropoff.pose.orientation.z = 0.0
dropoff.pose.orientation.w = 1.0

def joint_state_callback(msg: JointState):
    global joint_states
    joint_states = msg

def handle_place_req(req: PlaceRequest):
    global dropoff
    dropoff.header.stamp = rospy.Time.now()
    if joint_states.position[-1] == gripper_closed:
        
        # get hover position
        pos_hover = [dropoff.pose.position.x - 0.02, dropoff.pose.position.y, 0.0]
        q_hover = analyticalIK_lock4(pos_hover)

        # get dropoff position
        q_dropoff = analyticalIK_lock4([dropoff.pose.position.x, dropoff.pose.position.y, dropoff.pose.position.z])

        # define velocities
        q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5)),
                              JointTrajectoryPoint(positions=q_dropoff, velocities=q_dot, time_from_start=rospy.Duration(1.0))]
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        # drop object
        openGripper = CommandDuration(duration=200.0)
        openGripper.data = gripper_open
        gripperPub.publish(openGripper)

        rospy.sleep(1.0)

        # go to home position
        goal.trajectory.points = [JointTrajectoryPoint(positions=q_home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

    else:
        rospy.logerr("Gripper is not closed")

    return PlaceResponse(True, "Success")


if __name__ == "__main__":
    rospy.init_node("place_server")
    
    rospy.Service("/place", Place, handle_place_req)

    jointStateSub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    gripperPub = rospy.Publisher('/r_joint_controller/command_duration', CommandDuration, queue_size=1)
    trajectory_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    tflistener = tf2_ros.TransformListener(tf_buffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz

    rospy.spin()