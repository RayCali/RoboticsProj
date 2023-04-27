import numpy as np
from typing import List
import rospy
F = 10  # receptive field
mask: np.array = np.array([[0 for i in range(F)] for i in range(F)])