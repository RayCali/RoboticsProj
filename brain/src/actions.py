#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool
from msg_srv_pkg.srv import Request

class doExplore(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doExplore/mapping_and_planning/brain", Request)

class doLocalize(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doLocalize/memory/brain", Request)
    def tick(self):
        res = Action.tick(self)
        print("doLocalize: ", res)
        return res

# class doMoveToToy(Action):
#     def __init__(self) -> None:
#         super().__init__()
#         self.service = rospy.ServiceProxy("/srv/doMoveToToy/point_follower/brain", Request)
#     def tick(self):
#         res = super().tick()
#         print("doMoveToToy: ", res)
#         return res
    
class doMoveToGoal(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doMoveToGoal/point_follower/brain", Request)
    def tick(self):
        res = super().tick()
        print("doMoveToGoal: ", res)
        return res

class doPickup(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPickToy/pickup/brain", Request)
    def tick(self):
        res = super().tick()
        print("doPickup: ", res)
        return res

#comment
class doPlanPath(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlanpath/mapping_and_planning/brain", Request)
    def tick(self):
        res = super().tick()
        print("doPlanPath: ", res)
        return res
    

class doMoveAlongPath(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doMoveAlongPath/path_follower/brain", Request)
    def tick(self):
        res = super().tick()
        print("doMoveAlongPath: ", res)
class doPlace(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlaceToy/place/brain", Request)
    def tick(self):
        res = super().tick()
        print("doPlace: ", res)
        return res

class doPlanPathExplore(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlanpathExplore/mapping_and_planning/brain", Request)
    def tick(self):
        res = super().tick()
        print("doPlanPathExplore: ", res)
        return res