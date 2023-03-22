#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool

class ifAnchorDetected(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('AnchorDetected/srv', SetBool)
    


class ifFoundMatchingToyAndBox(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('FoundMatchingToyAndBox/srv', SetBool)

    

