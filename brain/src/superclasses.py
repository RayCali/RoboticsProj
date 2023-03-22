from array import array
from typing import List
from config import *


class Leaf:
    def __init__(self) -> None:
        pass
    def tick(self) -> int:
        return RUNNING

class And(Leaf):
    def __init__(self, children: List[Leaf]) -> None:
        super().__init__()
        self.children = array(Leaf, children)
    def tick(self) -> int:
        for child in self.children:
            childstatus = child.tick()
            if childstatus == RUNNING:
                return RUNNING
            elif childstatus == FAILURE:
                return FAILURE
        return SUCCESS

class Else(Leaf):
    def __init__(self, children: List[Leaf]) -> None:
        super().__init__()
        self.children = array(Leaf, children)
    def tick(self) -> int:
        for child in self.children:
            childstatus = child.tick()
            if childstatus == SUCCESS:
                return SUCCESS
            elif childstatus == RUNNING:
                return RUNNING
        return FAILURE