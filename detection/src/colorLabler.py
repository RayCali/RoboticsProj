#!/usr/bin/python3

from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
import math
import cv2

class ColorLabeler:
    def __init__(self) -> None:
        # initialize the colors dictionary, containing the color
        # name as the key and the RGB tuple as the value
        colors = OrderedDict({
            "red": (255, 0, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255)
        })
        # allocate memory for the L*a*b* image, then initialize
        # the color names list
        self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
        self.colorNames = []
        # loop over the colors dictionary
        for (i, (name, rgb)) in enumerate(colors.items()):
            # update the L*a*b* array and the color names list
            self.lab[i] = rgb
            self.colorNames.append(name)
        # convert the L*a*b* array from the RGB color space
        # to L*a*b*
        self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

    def label(self, center_colour):
        """
        Label the color of an object.

        Inputs:
            center_colour: colour of center of object in L*a*b* colour space.

        Output: 
            the name of the color.
        """
        # initialize the minimum distance found thus far
        minDist = (np.inf, None)

        # loop over the known L*a*b* color values
        for (i, row) in enumerate(self.lab):
            print("Lab {}: {}".format(i,row[0]))
            # compute the distance between the current L*a*b*
            # color value and the mean of the image
            d = math.sqrt((center_colour[0]-row[0][0])**2 + (center_colour[1]-row[0][1])**2 + (center_colour[2]-row[0][2])**2)
            print("Distance from {} is {}".format(self.colorNames[i], d))
            # if the distance is smaller than the current distance,
            # then update the bookkeeping variable
            if d < minDist[0]:
                minDist = (d, i)
            
        # return the name of the color with the smallest distance
        return self.colorNames[minDist[1]]