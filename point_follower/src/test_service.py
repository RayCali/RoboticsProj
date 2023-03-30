#!/usr/bin/env python3

from point_follower.srv import Node, Moveto, MovetoResponse, MovetoRequest
from detection.msg import objectPoseStampedLst
import rospy
from visualization_msgs.msg import Marker
seenanchor = False
#create node that calls Moveto service
def tracker(msg):
  global seenanchor
  if seenanchor:
    rospy.loginfo("Before wait for service")
    rospy.wait_for_service('/moveto')
    rospy.loginfo("Service found")
    s = rospy.ServiceProxy('/moveto', Moveto)
    if len(msg.PoseStamped) == 0:
      rospy.loginfo("No object detected")
      exit()
    resp1 = s(msg.PoseStamped[0])
    resp1.wait_for_result()
  
def anchorcallback(msg):
  global seenanchor
  seenanchor=True
if __name__ == '__main__':
    rospy.init_node('test_service')
    goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, tracker, queue_size=1) # has to be the pose of the postion we want to go to
    anchor_sub = rospy.Subscriber("/boundaries", Marker, anchorcallback, queue_size=1)
    print("Ready")
    rospy.spin()