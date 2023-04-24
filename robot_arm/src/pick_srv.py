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
from msg_srv_pkg.srv import Pick, PickRequest, PickResponse, Request, RequestResponse, RequestRequest
from utils import *

class Picker():

    def __init__(self) -> None:
        # rospy.Service("/pickup", Pick, self.handle_pickup_req)
        rospy.Service("/pickToy", Request, self.doPickupToy)
        rospy.Service("/isPicked", Request, self.isPicked)

        self.jointStateSub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.gripperPub = rospy.Publisher('/r_joint_controller/command_duration', CommandDuration, queue_size=1)
        self.trajectory_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        tflistener = tf2_ros.TransformListener(self.tf_buffer)
        tfbroadcaster = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10) # 10hz

        self.pickPose_pub = rospy.Publisher('/pick_pose', PoseStamped, queue_size=1)
        self.doPickSub = rospy.Subscriber('/pick_pose', PoseStamped, self.handle_pickup_req)

        self.joint_states = None
        self.pick_status = None
        self.running = False
        self.pickPose = None
        self.STATE = FAILURE

        self.pickPose_sub = rospy.Subscriber('/object_finalpose', PoseStamped, self.doSavePickPose)

    
    def doSavePickPose(self, msg: PoseStamped):
        self.pickPose = msg

    def isPicked(self, msg: RequestRequest):
        if self.pick_status == SUCCESS:
            return RequestResponse(SUCCESS)
        else:
            return RequestResponse(FAILURE)


    def doPickupToy(self, msg: RequestRequest):
        if not self.running:
            if self.pickPose is None or (rospy.Time.now().secs - self.pickPose.header.stamp.secs > 5):
                return RequestResponse(FAILURE)
            self.running = True
            self.STATE = RUNNING
            self.pickPose_pub.publish(self.pickPose)
            return RequestResponse(RUNNING)
        if self.running:
            if self.STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.STATE == SUCCESS:
                self.running = False
                return RequestResponse(SUCCESS)
            if self.STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)
    
    def joint_state_callback(self, msg: JointState):
        self.joint_states = msg


    def handle_pickup_req(self, msg: PoseStamped):
        rospy.loginfo("Let's do some picking!")

        if self.joint_states.position[-1] == gripper_open:

            # transform pose given in base_link to arm_base
            pose_stamped = msg
            print("BASELINK_POSE: ", pose_stamped)
            # pose_stamped.pose.position.x = 0.05
            stamp = pose_stamped.header.stamp

            try:
                self.tf_buffer.lookup_transform('arm_base', pose_stamped.header.frame_id, stamp, timeout=rospy.Duration(4.0))
                pose_base = self.tf_buffer.transform(pose_stamped, 'arm_base')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Could not get transform")
                return
            print("POSE_BASE: ", pose_base)
            if pose_base.pose.position.x < 0.15:
                return PickResponse(False, "Object too close to robot")
            # go to hover position
            pos_hover = [pose_base.pose.position.x - 0.02, pose_base.pose.position.y, 0.0]
            q_hover = analyticalIK_lock4(pos_hover)

            # go to desired position
            pos_pick = [pose_base.pose.position.x + 0.05, pose_base.pose.position.y, -0.1]
            q_pick = analyticalIK_lock4(pos_pick)

            q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]

            print("wait for server")
            self.trajectory_client.wait_for_server()
            print("Connected to server")
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
            goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5)),
                                    JointTrajectoryPoint(positions=q_pick, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
            
            # goal.trajectory.points = [JointTrajectoryPoint(positions=home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]

            print("Sending goal")
            self.trajectory_client.send_goal(goal)

            self.trajectory_client.wait_for_result()
            
            if self.trajectory_client.get_state() == GoalStatus.SUCCEEDED and True:
                # pick up object
                closeGripper = CommandDuration(duration=200.0)
                closeGripper.data = gripper_closed
                self.gripperPub.publish(closeGripper)

                rospy.sleep(1.0)

                # go to hover position
                goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=q_dot, time_from_start=rospy.Duration(0.5))]
                self.trajectory_client.send_goal(goal)
                self.trajectory_client.wait_for_result()

                # go to home position
                goal.trajectory.points = [JointTrajectoryPoint(positions=q_home, velocities=q_dot, time_from_start=rospy.Duration(2.0))]
                self.trajectory_client.send_goal(goal)
                self.trajectory_client.wait_for_result()

        else:
            self.STATE = FAILURE
            return 

        self.STATE = SUCCESS
        return 
    

if __name__ == "__main__":
    rospy.init_node("pickup")
    rospy.loginfo("Starting pickup node")
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        picker = Picker()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()