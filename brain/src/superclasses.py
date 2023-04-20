#!/usr/bin/python3
from array import array
from typing import List
from config import *
from rospy import loginfo, ServiceException
from msg_srv_pkg.srv import Request, RequestResponse
from utilities import ServiceReturnedRunningException


class Leaf:
    # In classical robotics this is called and execution node
    def __init__(self) -> None:
        pass
    def tick(self) -> int:
        return RUNNING
    def getStatusFromNum(num):
        num = int(num)
        if num == SUCCESS:
            return SUCCESS
        elif num == FAILURE:
            return FAILURE
        elif num == RUNNING:
            return RUNNING

    
"""
Some brainstorming:
    - The service is called in the tick function and expects an immediate response from the service (1,-1,0).
        This won't work since the service call blocks the rest.
    - The service call is blocking, so the tick function can't return until the service call returns.
    - Rather have a custom function in the class that checks the status of the action, calls the service and returns the status.
        The tick functiuon then just checks the status through a class variable without having to call the service or another function which might block.
        -----> this is good, I like this.
    - Question is where to define that class. In the package of the service or in the brain package?
    - Shutup co pilot, I'm thinking.
    - Orrrr, we check the status of the action in the tick function and call the service in the tick function depending on the status and return the status.
    - This is a bit of a hack, but it works. shut up copilot.
    

"""  


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