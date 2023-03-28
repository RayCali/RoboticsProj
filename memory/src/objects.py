#!/usr/bin/python3
import rospy
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import tf_conversions

class Movable:
    def __init__(self, pose: PoseStamped, name: str) -> None:
        self.pose: PoseStamped = pose
        self.name: str = None

class Toy(Movable):
    count = 0
    def __init__(self, pose: PoseStamped, name: str) -> None:
        super().__init__(pose, name)
        self.inBox = False

class Plushie(Toy):
    def __init__(self, pose: PoseStamped, name: str) -> None:
        super().__init__(pose, name)
        self.inBox = False
class Cube(Toy):
    def __init__(self, pose: PoseStamped, name: str) -> None:
        super().__init__(pose, name)
        self.inBox = False

class Ball(Toy):
    def __init__(self, pose: PoseStamped, name: str) -> None:
        super().__init__(pose, name)
        self.inBox = False

class Box(Movable):
    count = 0
    def __init__(self, pose: PoseStamped, name: str) -> None:
        super().__init__(pose, name)
        self.hasArucoMarker = False
