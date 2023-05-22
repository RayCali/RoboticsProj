#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
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
from msg_srv_pkg.srv import Request, RequestResponse, RequestRequest, PickPose, PickPoseResponse, PickPoseRequest
from utils import *


class PickAndPlace():
    def __init__(self) -> None:
        self.q_observe = [0.0, -0.24713861786666663, -1.0011208418666664, -1.801179757333333, 0.0, -1.7802358066666664]
        self.q_observe = [0.0, -0.261799, -1.309, -math.pi/2, 0.0]
        self.q_home = [0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0]
        self.q_dropoff = analyticalIK_lock4([0.2, 0.0, 0.0])
        self.q_dot = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_open = -1.7802358066666664
        self.gripper_closed = 0.0
        rospy.Service("/srv/doPickToy/pickup/memory", Request, self.doPickupToy)
        rospy.Service("/srv/doPlace/pickup/memory", Request, self.doPlaceToy)

        rospy.wait_for_service("/srv/getPickPose/arm_camera/pickup")
        self.getPickPose_srv = rospy.ServiceProxy("/srv/getPickPose/arm_camera/pickup", PickPose)

        self.jointStateSub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.gripperPub = rospy.Publisher('/r_joint_controller/command_duration', CommandDuration, queue_size=1)
        self.trajectory_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        self.tflistener = tf2_ros.TransformListener(self.tf_buffer)
        self.tfbroadcaster = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10) # 10hz

        self.pickPose_pub = rospy.Publisher('/pick_pose', PoseStamped, queue_size=1)
        self.pickPose_sub = rospy.Subscriber('/object_finalpose', PoseStamped, self.doSavePickPose)
        self.doPickSub = rospy.Subscriber('/pick_pose', PoseStamped, self.handle_pickup_req)

        self.place_pub = rospy.Publisher("/placeToy", Bool, queue_size=1)
        self.place_sub = rospy.Subscriber("/doPlace", Bool, self.doSaveIfPlace)
        self.doPlaceSub = rospy.Subscriber("/placeToy", Bool, self.handle_place_req)

        self.reset_sub = rospy.Subscriber("/RESET", Bool, self.handle_reset_req)

        self.joint_states = None
        self.running = False
        self.pickPose = PoseStamped()
        self.doPlace = False
        self.pick_STATE = FAILURE
        self.place_STATE= FAILURE


        rospy.sleep(1)
        T0E = forward_kinematics(list(self.joint_states.position))
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "arm_base"
        t.child_frame_id = "end_effector"

        t.transform.translation.x = T0E[0,3]
        t.transform.translation.y = T0E[1,3]
        t.transform.translation.z = T0E[2,3]
        q = tf_conversions.transformations.quaternion_from_matrix(T0E)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tfbroadcaster.sendTransform(t)

        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()


    def handle_reset_req(self, msg: Bool):
        if msg.data:
            self.pick_STATE = FAILURE
            self.place_STATE = FAILURE
            self.running = False
            self.doPlace = False
            self.pickPose = PoseStamped()


    def doSavePickPose(self, msg: PoseStamped):
        self.pickPose = msg

    def doSaveIfPlace(self, msg: Bool):
        self.doPlace = msg

    def joint_state_callback(self, msg: JointState):
        self.joint_states = msg

    def isPicked(self, msg: RequestRequest):
        if self.pick_STATE == SUCCESS:
            return RequestResponse(SUCCESS)
        else:
            return RequestResponse(FAILURE)


    def doPickupToy(self, msg: RequestRequest):
        if not self.running:
            if self.pickPose is None: # or (rospy.Time.now().secs - self.pickPose.header.stamp.secs > 5):
                return RequestResponse(FAILURE)
            self.running = True
            self.pick_STATE = RUNNING
            self.pickPose_pub.publish(self.pickPose)
            return RequestResponse(RUNNING)
        if self.running:
            if self.pick_STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.pick_STATE == SUCCESS:
                self.running = False
                rospy.loginfo("Resetting running to False")
                return RequestResponse(SUCCESS)
            if self.pick_STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)
            

    def doPlaceToy(self, msg:RequestRequest):
        if not self.running:
            if self.joint_states.position[5] == self.gripper_open:
                return RequestResponse(FAILURE)
            self.running = True
            self.place_STATE = RUNNING
            self.place_pub.publish(self.doPlace)
            return RequestResponse(RUNNING)
        if self.running:
            if self.place_STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.place_STATE == SUCCESS:
                self.running = False
                return RequestResponse(SUCCESS)
            if self.place_STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)


    def handle_pickup_req(self, msg: PoseStamped):
        rospy.loginfo("Let's do some picking!")

        if not (self.joint_states.position[-1] == self.gripper_open):

            openGripper = CommandDuration(duration=200.0)
            openGripper.data = self.gripper_open
            self.gripperPub.publish(openGripper)
            rospy.sleep(1.0)

        # move to observer position
        print("wait for server")
        self.trajectory_client.wait_for_server()
        print("Connected to server")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        goal.trajectory.points = [JointTrajectoryPoint(positions=self.q_observe, velocities=self.q_dot, time_from_start=rospy.Duration(0.5))]

        print("Sending goal")
        self.trajectory_client.send_goal(goal)

        self.trajectory_client.wait_for_result()

        # publish tf of end effector
        T0E = forward_kinematics(list(self.joint_states.position))
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "arm_base"
        t.child_frame_id = "end_effector"

        t.transform.translation.x = T0E[0,3]
        t.transform.translation.y = T0E[1,3]
        t.transform.translation.z = T0E[2,3]
        q = tf_conversions.transformations.quaternion_from_matrix(T0E)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tfbroadcaster.sendTransform(t)

        rospy.sleep(1.0)

        try:
            resp = self.getPickPose_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
            return
        
        if resp.success:
            # get pose of object in arm_base
            resp.pose_stamped.header.stamp = t.header.stamp
            alpha = resp.pose_stamped.pose.orientation.y
            print("ALPHA: ", alpha)
            try:
                rospy.loginfo("Waiting for transform from arm_base to object in frame: %s"%resp.pose_stamped.header.frame_id)
                self.tf_buffer.lookup_transform('arm_base', resp.pose_stamped.header.frame_id, t.header.stamp, timeout=rospy.Duration(5.0))
                rospy.loginfo("Goteeeeeem")
                pose_base = self.tf_buffer.transform(resp.pose_stamped, 'arm_base')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Could not get transform from arm_base to object in frame: %s"%resp.pose_stamped.header.frame_id)
                return

            print("POSE_BASE: ", pose_base)
            if pose_base.pose.position.x < 0.15:
                rospy.loginfo("Object too close to base, service call failed!!!")
                self.pick_STATE = FAILURE
                return
            if pose_base.pose.position.x > 0.22:
                rospy.loginfo("Object too far from base, service call failed!!!")
                # self.twist.linear.x = pose_base.pose.position.x - 0.18
                # self.pub_twist.publish(self.twist)
                self.pick_STATE = FAILURE
                return
            
            # go to hover position
            pos_hover = [pose_base.pose.position.x, pose_base.pose.position.y, 0.0]
            q_hover = analyticalIK_lock4(pos_hover, resp.theta, alpha)

            pos_pick = [pose_base.pose.position.x, pose_base.pose.position.y, -0.1]
            q_pick = analyticalIK_lock4(pos_pick, resp.theta, alpha)

            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
            goal.trajectory.points = [JointTrajectoryPoint(positions=q_hover, velocities=self.q_dot, time_from_start=rospy.Duration(0.5)),
                                      JointTrajectoryPoint(positions=q_pick, velocities=self.q_dot, time_from_start=rospy.Duration(1.0))]

            print("Sending goal")
            self.trajectory_client.send_goal(goal)

            self.trajectory_client.wait_for_result()
        else:
            rospy.logerr("Could not get pick pose")
            return
        
        if self.trajectory_client.get_state() == GoalStatus.SUCCEEDED:
            # pick up object
            closeGripper = CommandDuration(duration=1000.0)
            closeGripper.data = self.gripper_closed
            self.gripperPub.publish(closeGripper)

            rospy.sleep(1.0)

            # go to home position
            goal.trajectory.points = [JointTrajectoryPoint(positions=self.q_dropoff, velocities=self.q_dot, time_from_start=rospy.Duration(0.5)),
                                      JointTrajectoryPoint(positions=self.q_home, velocities=self.q_dot, time_from_start=rospy.Duration(1.0))]
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()

        if self.trajectory_client.get_result():
            rospy.loginfo("Picking successful!")
            self.pick_STATE = SUCCESS
            return
        
        self.pick_STATE = FAILURE
        return 
    

    def handle_place_req(self, msg:bool):
        if self.running:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
            goal.trajectory.points = [JointTrajectoryPoint(positions=self.q_dropoff, velocities=self.q_dot, time_from_start=rospy.Duration(0.5))]
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()

            # drop object
            openGripper = CommandDuration(duration=200.0)
            openGripper.data = self.gripper_open
            self.gripperPub.publish(openGripper)

            rospy.sleep(1.0)

            # go to home position
            goal.trajectory.points = [JointTrajectoryPoint(positions=self.q_home, velocities=self.q_dot, time_from_start=rospy.Duration(0.69))]
            self.trajectory_client.send_goal(goal)
            self.trajectory_client.wait_for_result()

            self.place_STATE = SUCCESS
        self.pick_STATE = FAILURE
        return

        
    

if __name__ == "__main__":
    rospy.init_node("pickup")
    rospy.loginfo("Starting pickup node aaaaaaaaa")
    
    try:
        rospy.loginfo("Creating PickAndPlace object")
        picker = PickAndPlace()
        rospy.loginfo("PickAndPlace object created")
    except rospy.ROSInterruptException:
        rospy.loginfo("Some error occurred.")
        pass
    
    rospy.spin()