#!/usr/bin/python3
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import tf_conversions
from typing import List

class Movable:
    def __init__(self, pose: PoseStamped, name: str,  id: int) -> None:
        self.poseStamped: PoseStamped = pose
        self.name: str = name
        self.id: int = id
        self.unreachable = False
    def __str__(self) -> str:
        return str(self.poseStamped.pose.position.x) + " " + str(self.poseStamped.pose.position.y) + " " + str(self.poseStamped.pose.position.z) + " " + str(self.id) + " " + str(self.name) + " " + str(self.unreachable)

class Toy(Movable):
    count = 0
    def __init__(self, pose: PoseStamped, name: str, id: int) -> None:
        super().__init__(pose, name, id)
        self.inBox = False


class Plushie(Toy):
    def __init__(self, pose: PoseStamped, name: str, id: int) -> None:
        super().__init__(pose, name, id)
class Cube(Toy):
    def __init__(self, pose: PoseStamped, name: str, id: int) -> None:
        super().__init__(pose, name, id)

class Ball(Toy):
    def __init__(self, pose: PoseStamped, name: str, id: int) -> None:
        super().__init__(pose, name, id)

class Box(Movable):
    count = 0
    def __init__(self, pose: PoseStamped, name: str, id: int) -> None:
        super().__init__(pose, name, id)
        self.hasArucoMarker = False
        self.objectLst: List[Toy] = []
