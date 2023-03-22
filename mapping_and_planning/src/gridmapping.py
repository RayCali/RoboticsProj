from gridobject import GridObject
import matplotlib.pyplot as plt
from typing import List
from geometry_msgs.msg import TransformStamped

class Mapper:
    def __init__(self) -> None:
        self.__setGridObject()
        self.frames_dict_keys = []
    
    def __setGridObject(self, resolution: int = 0.01) -> GridObject:
        self.go = GridObject(resolution)
    
    def doAnimate(self):
        x = self.go.getPILObject()
        plt.imshow(x)
        plt.show()
    
    def doInterpreteMSG(self, msg:TransformStamped, id:str="UNK") -> None:
        positions: List[List[int]] = []
        id = "UNK"
        # TODO: 
        # 2. figure out what type of object it is
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        positions = [
            [x,y],
            ]
        self.go.doPlace(positions=positions, id = id)
    
    def getKeyLogged(self, key: str) -> bool:
        return self.frames_dict_keys[key] is not None