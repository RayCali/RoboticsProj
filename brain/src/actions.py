#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool
from msg_srv_pkg.srv import Request

class doExplore(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("Explore/srv", Request)

class doLocalize(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("doLocalize", Request)
    def tick(self):
        res = Action.tick(self)
        return res

class doPickUp(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/pickup", Request)

class doMoveToGoal(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("doMoveToGoal", Request)

class doMoveToToy(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/moveToToy", Request)
    def tick(self):
        print("doMoveToToy")
        res = super().tick()
        return res
