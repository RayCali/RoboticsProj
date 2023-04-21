#!/usr/bin/python3
from array import array
from typing import List
from config import *
from rospy import loginfo, ServiceException
from msg_srv_pkg.srv import Request, RequestResponse
from Leaf import Leaf

    
class ServiceReturnedRunningException(Exception):
    def __init__(self, message) -> None:
        super().__init__(message)
        self.message = message



class Action(Leaf):
    # In classical robotics this is called and execution node
    def __init__(self) -> None:
        super().__init__()
    def tick(self):
        try:
            res: RequestResponse = self.service()
        except ServiceException as e:
            loginfo(e)
            return FAILURE
        return self.getStatusFromNum(res.success)

class Condition(Leaf):
    # In classical robotics this is called and execution node
    def __init__(self) -> None:
        super().__init__()
    def tick(self):
        try:
            res: RequestResponse = self.service()
        except ServiceException as e:
            loginfo(e)
            return FAILURE
        if self.getStatusFromNum(res.success)==RUNNING:
            raise ServiceReturnedRunningException("Service returned a RUNNING state to a condition node. This is not allowed.")
        
        return self.getStatusFromNum(res.success)