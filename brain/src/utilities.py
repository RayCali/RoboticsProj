#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool


class And(Leaf):
    # In classical robotics this is called a sequence node
    def __init__(self, children: List[Leaf]) -> None:
        super().__init__()
        self.children = children
    def tick(self) -> int:
        for child in self.children:
            childstatus = child.tick()
            if childstatus == RUNNING:
                return RUNNING
            elif childstatus == FAILURE:
                return FAILURE
        return SUCCESS

class Or(Leaf):
    # In classical robotics this is called a fallback node
    # The wikipedia article dubs it the selector node
    def __init__(self, children: List[Leaf]) -> None:
        super().__init__()
        self.children = children
    def tick(self) -> int:
        for child in self.children:
            childstatus = child.tick()
            if childstatus == SUCCESS:
                return SUCCESS
            elif childstatus == RUNNING:
                return RUNNING
        return FAILURE

class IsListException(Exception):
    def __init__(self, message) -> None:
        super().__init__(message)
        self.message = message

class Root:
    def __init__(self, child) -> None:
        self.child = child
        if not isinstance(child, And):
            raise IsListException("The child of a Root object should be a single And object")
    def tick(self) -> int:
        return self.child.tick()
