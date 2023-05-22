#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool
from msg_srv_pkg.srv import Request

class doSelect(Action):
    def __init__(self) -> None:
        super().__init__()
        #self.service = rospy.ServiceProxy("/srv/doExplore/mapping_and_planning/brain", Request)
        self.service = rospy.ServiceProxy("/srv/doSelectExplorationGoal/memory/brain", Request)
        self.verbose = True

class doLocalize(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doLocalize/memory/brain", Request)

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
        self.verbose = True


class doPickup(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPickToy/memory/brain", Request)

        self.verbose = True
        # Staffan is to remap this service below to provide for the memory
        #self.service = rospy.ServiceProxy("/srv/doPickToy/pickup/brain", Request)



class doPlanPathExplore(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlanpath/memory/brain", Request)
        self.verbose = True
    
class doPlanPathToy(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlanpathToy/memory/brain", Request)
        self.verbose = True

        # Staffan is to remap this service below to provide for the memory 
        #self.service = rospy.ServiceProxy("/srv/doPlanpathToy/mapping_and_planning/brain", Request)
    
class doPlanPathBox(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlanpathBox/memory/brain", Request)
        self.verbose = True

class doPlanExplorationPath(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlanExplorationPath/memory/brain", Request)
        self.verbose = True
        #self.service = rospy.ServiceProxy("/srv/doPlanExplorationPath/mapping_and_planning/brain", Request)
        
class doMoveAlongPathGlobal(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doMoveAlongPathGlobal/path_follower_global/brain", Request)
        self.verbose = True

class doMoveAlongPathToy(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doMoveAlongPathToyLocal/memory/brain", Request)
        self.verbose = True
        #self.service = rospy.ServiceProxy("/srv/doMoveAlongPathToyLocal/path_follower_local/brain", Request)
    
class doMoveAlongPathBox(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doMoveAlongPathBoxLocal/memory/brain", Request)
        self.verbose = True

class doPlace(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doPlace/memory/brain", Request)
        self.verbose = True
    

class doReset(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doReset/memory/brain", Request)
        self.verbose = True
    
class stopExplore(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/stopExplore/mapping_and_planning/brain", Request)
        self.verbose = True

class doMoveAlongPathGlobal(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("/srv/doMoveAlongPathGlobal/memory/brain", Request)
        self.verbose = True
