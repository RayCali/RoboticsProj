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
    def tick(self):
        res = Condition.tick(self)
        print("isNotPair? ", res)
        return res

class isFound(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isFound/memory/brain', Request)

class isPlanned(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPlanned/memory/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isPlanned? ", res)
        return res

class isAtToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isAtToy/memory/brain', Request)
        # Rayan is to remove this functionality below from the path_follower
        #self.service = rospy.ServiceProxy('/srv/isAtToy/path_follower_local/brain', Request)
        
    def tick(self):
        res = Condition.tick(self)
        print("atToy? ", res)
        return res
    
class isPicked(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPicked/memory/brain', Request)

    def tick(self):
        res = Condition.tick(self)
        print("isPicked?, ", res)
        return res

class isMovingToToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isMovingToToy/memory/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isMovingToToy?, ", res)
        return res
class isNotExploring(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isNotExploring/path_follower_global/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isNotExploring?, ", res)
        return res

class isAtBox(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isAtBox/memory/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isAtBox?, ", res)
        return res

class isPlannedBox(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPlannedBox/memory/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isPlannedBox?, ", res)
        return res

class isPlaced(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPlaced/memory/brain', Request)
    def tick(self):
        res = Condition.tick(self)
        print("isPlaced?, ", res)
        return res