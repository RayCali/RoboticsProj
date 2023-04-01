#!/usr/bin/python3

from superclasses import *
import rospy
from msg_srv_pkg.srv import Request, RequestResponse

class isLocalized(Condition):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy('isLocalized', Request)

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