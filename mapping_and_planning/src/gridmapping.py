from gridobject import GridObject
import matplotlib.pyplot as plt
from typing import List

class Mapper:
    def __init__(self) -> None:
        self.__setGridObject()
    
    def __setGridObject(self, resolution: int = 0.01) -> GridObject:
        self.go = GridObject(resolution)
    
    def doAnimate(self):
        x = self.go.getPILObject()
        plt.imshow(x)
        plt.show()
    
    def doInterpreteMSG(self, msg) -> None:
        positions: List[List[int]] = []
        id = "UNK"
        # TODO: 
        # 1. add functionality that deciphers a msg into 
        # a list of positions within the grid
        # 2. figure out what type of object it is
        
        self.go.doPlace(positions=positions, id = id)
        return