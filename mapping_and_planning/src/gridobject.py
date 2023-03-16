import numpy as np
import matplotlib.pyplot as plt
class GridObject:
    def __init__(self, resolution: int) -> None:
        L = 10
        N = int(L / resolution) #ir stands for Internal Representation
        self.__ir = np.array([[(0, 255, 255, 255) for i in range(N)] for i in range(N)], dtype=np.uint8)
        self.__drawBox()
    
    def __drawBox(self):
        for i in range(500,600,1):
            for ii in range(500,600,1):
                self.__ir[i,ii] = (1,0,0,0)


    def getPILObject(self) -> np.array:
        return self.__ir[:,:,1:]

    

