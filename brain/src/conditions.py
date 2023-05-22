#!/usr/bin/python3

from superclasses import *
import rospy
from msg_srv_pkg.srv import Request, RequestResponse

class isLocalized(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isLocalized/memory/brain', Request)
        self.verbose = True

class isNotPair(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isNotPair/memory/brain', Request)
        self.verbose = True

class isFound(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isFound/memory/brain', Request)
        self.verbose = True

class isPlanned(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPlanned/memory/brain', Request)
        self.verbose = True

class isAtToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isAtToy/memory/brain', Request)
        self.verbose = True
        # Rayan is to remove this functionality below from the path_follower
        #self.service = rospy.ServiceProxy('/srv/isAtToy/path_follower_local/brain', Request)
        
    
class isPicked(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPicked/memory/brain', Request)
        self.verbose = True

class isMovingToToy(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.verbose = True
        self.service = rospy.ServiceProxy('/srv/isMovingToToy/memory/brain', Request)
class isNotExploring(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isNotExploring/path_follower_global/brain', Request)
        self.verbose = True

class isAtBox(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isAtBox/memory/brain', Request)
        self.verbose = True

class isPlannedBox(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPlannedBox/memory/brain', Request)
        self.verbose = True

class isPlaced(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isPlaced/memory/brain', Request)
        self.verbose = True

class isSelected(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isGoalSelected/memory/brain', Request)
        self.verbose = True

class isExplorationPathPlanned(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('/srv/isExplorationPathPlanned/memory/brain', Request)
        self.verbose = True

