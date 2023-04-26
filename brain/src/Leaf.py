#!/usr/bin/python3
from typing import List
from config import *
from rospy import loginfo

class Leaf:
    # In classical robotics this is called and execution node
    def __init__(self) -> None:
        pass
    def tick(self) -> int:
        return RUNNING
    def getStatusFromNum(self, num):
        num = int(num)
        if num == SUCCESS:
            return SUCCESS
        elif num == FAILURE:
            return FAILURE
        elif num == RUNNING:
            return RUNNING