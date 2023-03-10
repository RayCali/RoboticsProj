from gridobject import GridObject
class Mapper:
    def __init__(self) -> None:
        self.go = self.__getGridObject()
    
    def __getGridObject(self, resolution: int = 0.01):
        return GridObject(resolution)
    