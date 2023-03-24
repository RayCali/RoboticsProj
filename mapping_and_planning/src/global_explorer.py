#!/usr/bin/python3
from gridmapping import *
# https://cs231n.github.io/convolutional-networks/
F = 10  # receptive field
S = 10   # stride
P = 0   # padding
mask: np.array = (
    [[2 for i in range(F)] for i in range(F)]
)

def getValues(m: Map) -> np.array:
    W = m.grid.info.width
    H = m.grid.info.height
    for w in range(0, W - mask.shape[1], S):
        for h in range(H - mask.shape[1], S):
            area = m[w:w+F,h:h+F]
            print(area)
    return mask
