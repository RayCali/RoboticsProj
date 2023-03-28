#!/usr/bin/python3
import rospy
import numpy as np
from numpy import dot, array, 
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from detection.msg import objectPoseStampedLst
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import tf_conversions
from objects import Plushie, Cube, Ball, Box, Movable
from typing import Dict
from utilities import normalized


class Mem:
    def __init__(self):
        self.objects: Dict[Movable]= {}
        self.plushies: Dict[Plushie] = {}
        self.cubes: Dict[Cube] = {}
        self.balls: Dict[Ball] = {}
        self.boxes: Dict[Box] = {}
        self.id2Object = {
           0 : "Binky",
           1 : "Hugo",
           2 : "Slush",
           3 : "Muddles",
           4 : "Kiki",
           5 : "Oakie",
           6 : "Cube",
           7 : "Ball",
           8 : "Box",
        }
        self.detection_sub = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.__doStoreAllDetectedObjects)
        self.xThreshold = 0.2
        self.yThreshold = 0.2

    def __putObject(self, pose: PoseStamped, id: int):
        id = int(id)
        object: Movable
        objectType = None
        correctDict = None
        if id in [0, 1, 2, 3, 4, 5]:
            objectType = Plushie
            correctDict = self.plushies
        elif id==6:
            objectType = Cube
            correctDict= self.cubes
        elif id==7:
            objectType = Ball
            correctDict= self.balls
        elif id==8:
            objectType = Box
            correctDict = self.boxes
        else:
            raise Exception("Invalid object ID: " % str(id))
        name = self.id2Object[id] + "_" + str(objectType.count)
        objectType.count += 1
        object = objectType(pose=pose, name=name)
        self.objects[name] = object
        correctDict[name] = object


    def __doStoreAllDetectedObjects(self, msg: objectPoseStampedLst):
        for pose, id in zip(msg.PoseStamped, msg.object_class):
            self.__putInDictsIfNotAlreadyIn(pose,id)
    
    def __doStoreAllDetectedObjects(self, pose: PoseStamped, id: int):
        for object in self.objects:
            if abs(object.x - pose.pose.position.x) < self.xThreshold or abs(object.y - pose.pose.position.y) < self.yThreshold:
                return
        self.__putObject(pose, id)
        

            

            


if __name__ == "__main__":

    rospy.init_node("memory")
    m = Mem()
    rospy.spin()    
