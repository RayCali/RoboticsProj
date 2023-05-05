#!/usr/bin/python3

from superclasses import *
import rospy
from msg_srv_pkg.srv import Request, RequestResponse

class isLocalized(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isLocalized/memory/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        return res

class isNotPair(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isNotPair/memory/brain', Request)

class isFound(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isFound/memory/brain', Request)
    
class isAtToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isAtToy/point_follower/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("atToy? ", res)
        return res
    
class isPicked(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPicked/pickup/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isPicked?, ", res)
        return res
