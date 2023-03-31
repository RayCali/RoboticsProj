import numpy as np
def normalized(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm
