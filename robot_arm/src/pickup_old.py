#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from msg_srv_pkg.msg import objectPoseStampedLst
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from msg_srv_pkg.srv import Pick, Place

pose= None
joint_states = None
joint_state_stamp = None
pickSuccess = False

def pose_callback(msg: PoseStamped):
    global pose
    pose = msg

def joint_state_callback(msg: JointState):
    global joint_states, joint_state_stamp
    joint_state_stamp = msg.header.stamp
    joint_states = msg


def call_pickup_callback(msg: Bool):
    global pickSuccess
    if msg.data == True:
        print("Waiting for service 'pickup'...")
        rospy.wait_for_service('/pickup')
        pickup = rospy.ServiceProxy('/pickup', Pick)

        print("Try calling service...")
        if not pickSuccess:
            try: 
                resp = pickup(pose)
                print(resp.msg)
                rospy.sleep(5.0)
                pickSuccess = True
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
    
def call_place_callback(msg: Bool):
    if msg.data == True:
        print("Waiting for service 'place'...")
        rospy.wait_for_service('/place')
        place = rospy.ServiceProxy('/place', Place)

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
        test_pose.header.frame_id = "arm_base"
        test_pose.header.stamp = rospy.Time.now()
        test_pose.pose.position.x = 0.16
        test_pose.pose.position.y = 0.0
        test_pose.pose.position.z = -0.035
        test_pose.pose.orientation.x = 0.0
        test_pose.pose.orientation.y = 0.0
        test_pose.pose.orientation.z = 0.0
        test_pose.pose.orientation.w = 1.0
        posePub.publish(test_pose)

        # if we have a pose (and later are in position), send command to pickup
        if pose:
            print("Calling pickup service...")
            callPickupPub.publish(Bool(data=True))
            rospy.sleep(5.0)
            if pickSuccess:
                callPlacePub.publish(Bool(data=True))
        rospy.sleep(0.1)