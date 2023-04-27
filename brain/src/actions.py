#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool
from msg_srv_pkg.srv import Request

# class doExplore(Action):
#     def __init__(self) -> None:
#         super().__init__()
#         self.service = rospy.ServiceProxy("doExplore", Request)
#         res = super().tick()
#         print("doExplore: ", res)
#         return res

class doLocalize(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("doLocalize", Request)
    def tick(self):
        res = Action.tick(self)
        return res

class doMoveToGoal(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("doMoveToGoal", Request)

class doMoveToToy(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/moveToToy", Request)
    def tick(self):
        res = super().tick()
        print("doMoveToToy: ", res)
        return res

class doPickup(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/pickToy", Request)
    def tick(self):
        print("doPickup")
        res = super().tick()
        return res
