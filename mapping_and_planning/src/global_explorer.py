#!/usr/bin/python3
import numpy as np
from typing import List
import rospy
from gridmapping import Map
# https://cs231n.github.io/convolutional-networks/
F = 10  # receptive field
S = 10   # stride
P = 0   # padding


# Leaving the mask here to illustrate how it looks, we can choose other values
mask: np.array = np.array([ [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

mask: np.array = np.array([[0 for i in range(F)] for i in range(F)])



def f(mask_value: int, matrix_value: int) -> int:
    if mask_value == matrix_value:
        return 10
    elif matrix_value != 2:
        return 1
    else:
        return 0
def heuristic(matrix: np.array, mask: np.array, width: int, height: int) -> int:
    area_value = 0
    for h in range(F):
        for w in range(F):
            area_value += f(mask[h,w], matrix[h,w])

    return area_value

def getMostValuedCell(matrix: np.array, width: int, height: int) -> List[int]:
    heuiristic_values = getHeuristicMap(matrix, mask, width, height)
    print(heuiristic_values)
    most_valued_cell = [0, 0, 0]
    for cell in heuiristic_values:
        if cell[0] > most_valued_cell[0]:
            most_valued_cell = cell
    return most_valued_cell

def getHeuristicMap(matrix: np.array, mask: np.array, W: int, H: int) -> np.array:
    heuiristic_values = []
    for h in range(0, H - F, S):
        for w in range(0, W - F, S):
            m = matrix[h:h+F, w:w+F]
            cell_value = heuristic(m, mask, W, H)
            heuiristic_values.append([cell_value, h, w])            
    return heuiristic_values

def getMaskandMatrix():
    global mask, mask_value, matrix, matrix_value
    mask = np.array([[0 for i in range(F)] for i in range(F)])
    mask = np.array([z+1 for z in range(F)] for y in range(F))
    matrix = np.array([[0 for i in range(11)] for i in range(11)])
    matrix_value = sum(matrix)
    mask_value = sum(mask)
    return mask, mask_value, matrix, matrix_value

if __name__ == "__main__":
    
    rospy.init_node("global_test")
    rospy.loginfo("Global explorer started")
    width = height = 11
    getMaskandMatrix()
    f(mask_value, matrix_value)
    getMostValuedCell(matrix, width, height)
