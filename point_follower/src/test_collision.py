#!/usr/bin/env python3

from point_follower.srv import Node, NodeRequest, NodeResponse
from detection.msg import objectPoseStampedLst
from geometry_msgs.msg import PoseStamped
import rospy

#create node that calls Node service
def position(msg):
  rospy.loginfo("Waiting for service")
  rospy.wait_for_service('/no_collision')
  rospy.loginfo("Service found")
  s = rospy.ServiceProxy('/no_collision', Node)
  if len(msg) == 0:
    rospy.loginfo("No obstacles detected")
    exit()
  resp1 = goal(msg.x)
  resp1.wait_for_result()
#   try:
#     rospy.loginfo("Pickup service called")
#   except rospy.ServiceException as e:
#     rospy.loginfo("Service call failed: %s"%e)
  

if __name__ == '__main__':
    rospy.init_node('test_collision_service')
    goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, position, queue_size=1)# has to be the pose of the postion we want to go to
    print("Ready")
    rospy.spin()