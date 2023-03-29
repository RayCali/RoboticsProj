#!/usr/bin/env python3

from point_follower.srv import Node, Moveto, MovetoResponse, MovetoRequest
from detection.msg import objectPoseStampedLst
import rospy

#create node that calls Moveto service
def tracker(msg):
  rospy.loginfo("Before wait for service")
  rospy.wait_for_service('/moveto')
  rospy.loginfo("Service found")
  s = rospy.ServiceProxy('/moveto', Moveto)
  if len(msg.PoseStamped) == 0:
    rospy.loginfo("No object detected")
    exit()
  resp1 = s(msg.PoseStamped[0])
  resp1.wait_for_result()
  

if __name__ == '__main__':
    rospy.init_node('test_service')
    goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, tracker, queue_size=1) # has to be the pose of the postion we want to go to
    print("Ready")
    rospy.spin()