#!/usr/bin/python3

from superclasses import *
import rospy
from msg_srv_pkg.srv import Request, RequestResponse

class isLocalized(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/isLocalized', Request)
    def tick(self):
        res = Condition.tick(self)
        return res


class isNotPair(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('isnotpair', Request)

class isPicked(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('isPicked', Request)

class isInFrontToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('isInFrontToy', Request)


class isFound(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/isFound', Request)
    def tick(self):
        res = Condition.tick(self)
        return res
    
class isAtToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/atToy', Request)
    def tick(self):
        res = Condition.tick(self)
        print("atToy? ", res)
        return res
    
class isPicked(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/isPicked', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isPicked?, ", res)
        return res
    
class isExplored(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/isExplored', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isExplored?, ", res)
        return res
