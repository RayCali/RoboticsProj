#!/usr/bin/env python3

from msg_srv_pkg.srv import Node, Moveto, MovetoResponse, MovetoRequest, Pick, PickRequest, PickResponse
from msg_srv_pkg.msg import objectPoseStampedLst
from geometry_msgs.msg import PoseStamped
import rospy
from visualization_msgs.msg import Marker


seenanchor = False
pickPose = None


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
    for i in range (len(msg.object_class)):
      if msg.object_class[i] != "box":
        resp1 = s(msg.PoseStamped[i])
        rospy.loginfo(resp1)
        if resp1.success:
          rospy.loginfo("Start picking!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
          rospy.wait_for_service("/pickup")
          pickService = rospy.ServiceProxy("/pickup", Pick)
          if pickPose is not None:
            try:
              rospy.sleep(2)
              pickResp = pickService(pickPose)
              rospy.loginfo(pickResp)
            except rospy.ServiceException as e:
              print("Service call failed: %s"%e)
        break


def pickPoseCallback(msg):
  global pickPose
  pickPose = msg


def anchorcallback(msg):
  global seenanchor
  seenanchor=True
if __name__ == '__main__':
    rospy.init_node('test_service')
    goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, tracker, queue_size=1) # has to be the pose of the postion we want to go to
    anchor_sub = rospy.Subscriber("/boundaries", Marker, anchorcallback, queue_size=1)
    pickPoseSub = rospy.Subscriber("/object_finalpose", PoseStamped, pickPoseCallback, queue_size=1)
    print("Ready")
    rospy.spin()