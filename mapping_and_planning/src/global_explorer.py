#!/usr/bin/python3
from gridmapping import *
from typing import List
# https://cs231n.github.io/convolutional-networks/
F = 10  # receptive field
S = 10   # stride
P = 0   # padding
mask: np.array = np.array(
    [[2 for i in range(F)] for i in range(F)]
)

def getMostValuedCell(matrix: np.array, width: int, height: int) -> List[int, int]:
    getHeuristicMap(matrix, width, height)

def getHeuristicMap(matrix: np.array, W: int, H: int) -> np.array:
    for h in range(H - F, S):
        for w in range(0, W - F, S):

            area = matrix[w:w+F,h:h+F]
            print(area)
    return mask
