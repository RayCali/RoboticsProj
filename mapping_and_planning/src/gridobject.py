import numpy as np
import matplotlib.pyplot as plt
from typing import List

class GridObject:
    def __init__(self, resolution: int, devel=True) -> None:
        L = 10
        N = int(L / resolution) #ir stands for Internal Representation
        self.__ir = np.array([[(0, 255, 255, 255) for i in range(N)] for i in range(N)], dtype=np.uint8)
        self.__id2info = {
            "UNK": (0, 255, 255, 255),
            "OBS": (1, 0, 0, 0),
            "BOX": (2, 0, 0, 255),
            "TOY": (3, 0, 255, 0),
        }
        if devel:
            self.__drawBox()

    def __drawBox(self):
        l = []
        for i in range(500,600,1):
            for ii in range(500,600,1):
                l.append([i,ii])
        self.doPlace(positions = l, id="OBS")

        l = []
        lower = 100
        upper = 200
        for i in range(lower,upper):
            l.append([i,lower])
            l.append([lower,i])
            l.append([upper,i])
            l.append([i,upper])
        
        self.doPlace(positions=l, id="BOX")

    def getPILObject(self) -> np.array:
        return self.__ir[:,:,1:]
    
    def doPlace(self, positions: List[List[int]], id = "UNK"):
        for p in positions:
            self.__ir[p[0], p[1]] = self.__id2info[id]
    

