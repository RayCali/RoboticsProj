import numpy as np
import matplotlib.pyplot as plt
class GridObject:
    def __init__(self, resolution: int) -> None:
        L = 10
        N = int(L / resolution)
        self.__ir = np.zeros((N,N)) #i.r.  stands for Internal Representation
        

