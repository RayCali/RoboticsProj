#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Trigger


pose_base = None
pose_stamp = None
joint_states = None
joint_state_stamp = None

def pose_callback(msg: PoseStamped):
    global pose_base, pose_stamp
    pose_stamp = msg.header.stamp
    pose_base = msg.pose

def joint_state_callback(msg: JointState):
    global joint_states, joint_state_stamp
    joint_state_stamp = msg.header.stamp
    joint_states = msg


def call_pickup_callback(msg: Bool):
    if msg.data == True:
        print("Waiting for service 'pickup'...")
        rospy.wait_for_service('/pickup')
        pickup = rospy.ServiceProxy('/pickup', Trigger)

        print("Try calling service...")
        try: 
            resp = pickup()
            print(resp.message)
            rospy.sleep(5.0)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def call_place_callback(msg: Bool):
    if msg.data == True:
        print("Waiting for service 'place'...")
        rospy.wait_for_service('/place')
        place = rospy.ServiceProxy('/place', Trigger)

        print("Try calling service...")
        try: 
            resp = place()
            print(resp.message)
            rospy.sleep(5.0)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('pickup_client')

    poseSub = rospy.Subscriber('/detection/pose', PoseStamped, pose_callback)
    jointStateSub = rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    callPickupSub = rospy.Subscriber('/call_pickup', Bool, call_pickup_callback)
    callPickupPub = rospy.Publisher('/call_pickup', Bool, queue_size=10)
    callPlaceSub = rospy.Subscriber('/call_place', Bool, call_place_callback)
    callPlacePub = rospy.Publisher('/call_place', Bool, queue_size=10)
    posePub = rospy.Publisher('/detection/pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        test_pose = PoseStamped()
        test_pose.header.frame_id = "base_link"
        test_pose.header.stamp = rospy.Time.now()
        test_pose.pose.position.x = 0.08
        test_pose.pose.position.y = 0.0
        test_pose.pose.position.z = -0.035
        test_pose.pose.orientation.x = 0.0
        test_pose.pose.orientation.y = 0.0
        test_pose.pose.orientation.z = 0.0
        test_pose.pose.orientation.w = 1.0
        posePub.publish(test_pose)

        # if we have a pose (and later are in position), send command to pickup
        if pose_base:
            callPickupPub.publish(Bool(data=True))
            rospy.sleep(15.0)
            callPlacePub.publish(Bool(data=True))
        rospy.sleep(0.1)