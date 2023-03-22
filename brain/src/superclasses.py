#!/usr/bin/python3
from array import array
from typing import List
from config import *
from rospy import loginfo, ServiceException
from std_srvs.srv import SetBool


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

    
    


class Action(Leaf):
    # In classical robotics this is called and execution node
    def __init__(self) -> None:
        super().__init__()
    def tick(self):
        try:
            res: SetBool = self.service()
        except ServiceException as e:
            loginfo(e)
            return FAILURE
        return self.getStatusFromNum(res.message)

class Condition(Leaf):
    # In classical robotics this is called and execution node
    def __init__(self) -> None:
        super().__init__()
    def tick(self):
        try:
            res: SetBool = self.service()
        except ServiceException as e:
            loginfo(e)
            return FAILURE
        return self.getStatusFromNum(res.message)