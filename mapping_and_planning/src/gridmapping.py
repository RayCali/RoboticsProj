from gridobject import GridObject
import matplotlib.pyplot as plt
import numpy as np

class Mapper:
    def __init__(self) -> None:
        self.go = self.__getGridObject()
    
    def __getGridObject(self, resolution: int = 0.01) -> GridObject:
        return GridObject(resolution)
    
    def doAnimate(self):
        x = self.go.getPILObject()
        print(x)
        print(x.shape)
        
        plt.imshow(x)
        plt.show()