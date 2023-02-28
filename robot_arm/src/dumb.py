#!/usr/bin/env python3

import rospy
from hiwonder_servo_msgs.msg import CommandDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib


if __name__ == "__main__":
    rospy.init_node("dumb")
    pub = rospy.Publisher("/joint1_controller/command_duration", CommandDuration, queue_size=1)
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.sleep(1)
    # msg = CommandDuration()
    # msg.duration = 1000.0
    # msg.data = 0.0
    # pub.publish(msg)
    # rospy.spin()

    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
    goal.trajectory.points = [JointTrajectoryPoint(positions=[0.0, 0.5235987666666666, -1.361356793333333, -1.7592918559999997, 0.0, -1.7802358066666664], velocities=[0.0, 0.0, 0.0, 0.0, 0.0], time_from_start=rospy.Duration(2.5))]
    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_result())

    rospy.spin()