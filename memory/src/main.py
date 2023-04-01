#!/usr/bin/python3
import rospy
import numpy as np
from numpy import dot, array
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from aruco_msgs.msg import MarkerArray, Marker
import matplotlib.pyplot as plt
import tf_conversions
from objects import Plushie, Cube, Ball, Box, Movable, Toy
from typing import Dict
from utilities import normalized
from msg_srv_pkg.msg import objectPoseStampedLst
from msg_srv_pkg.srv import Moveto, MovetoResponse, Request, RequestResponse
from config import SUCCESS, RUNNING, FAILURE

class Memory:
    def __init__(self):
        self.objects: Dict[Movable]= {}
        self.plushies: Dict[Plushie] = {}
        self.cubes: Dict[Cube] = {}
        self.balls: Dict[Ball] = {}
        self.toys: Dict[Toy] = {}
        self.boxes: Dict[Box] = {}
        self.id2Object = {
           0 : "Binky",
           1 : "Hugo",
           2 : "Slush",
           3 : "Muddles",
           4 : "Kiki",
           5 : "Oakie",
           6 : "cube",
           7 : "ball",
           8 : "box",
        }
        self.object2Id = {
            "Binky"  : 0,
            "Hugo"   : 1,
            "Slush"  : 2,
            "Muddles": 3,
            "Kiki"   : 4,
            "Oakie"  : 5,
            "cube"   : 6,
            "ball"   : 7,
            "box"    : 8
        }
        self.arucoId2Box = {
            0 : "Box_Plushies",
            1 : "Box_Balls",
            2 : "Box_Cubes",
            500 : "Anchor"
        }

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.detection_sub = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.__doStoreAllDetectedObjects)
        self.aruco_sub = rospy.Subscriber("/aruco_all/aruco/markers", MarkerArray, self.__doStoreAllBoxesWAruco)

        self.moveto_srv = rospy.Service("moveto", Moveto, self.doMoveTo)
        self.isLocalized_srv = rospy.Service("isLocalized", Request, self.getIsLocalized)
        self.doLocalize_srv = rospy.Service("doLocalize", Request, self.doLocalize)

        
        self.isnotpair_srv = rospy.Service("notpair", Request, self.getNotPair)

        self.ispicked_srv = rospy.Service("ispicked", Request, self.getIsPicked)
        self.isInFrontToy = rospy.Service("isInFrontToy", Request, self.getIsInFrontToy)
        self.pathPlanner_proxy = rospy.ServiceProxy("pathPlanner", Moveto)
        
        self.movingToTargetToy = False
        self.targetToy: Toy = None
        self.targetHasBecomeInvalid = False
        self.xThreshold = 0.10
        self.yThreshold = 0.10

        
    
    def getIsInFrontToy(self, req):
        InFrontThreshold_x = 0.15
        InFrontThreshold_y = 0.15
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            return RequestResponse(FAILURE)
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        if abs(x - self.targetToy.pose.position.x) < InFrontThreshold_x and abs(y - self.targetToy.pose.position.y) < InFrontThreshold_y:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)


    
    def getIsPicked(self, req):
        return RequestResponse(FAILURE)
    def getNotPair(self, req):
        # TODO: check if there is a valid box-object pair in the memory and return FAILURE if there IS and SUCCESS if ther IS NOT.
        # we have a pair if
        # 1) the box object has an aruco marker so we can identify which box it is
        # 2) the dictionary of the object class that the box belongs to is not empty
        for key in self.boxes:
            box = self.boxes[key]
            if box.hasArucoMarker:
                if box.name == "Box_Plushies":
                    if len(self.plushies) > 0:
                        return RequestResponse(FAILURE)
                elif box.name == "Box_Balls":
                    if len(self.balls) > 0:
                        return RequestResponse(FAILURE)
                elif box.name == "Box_Cubes":
                    if len(self.cubes) > 0:
                        return RequestResponse(FAILURE)
        
        return RequestResponse(SUCCESS)
    
    def doLocalize(self, req):
        return RequestResponse(RUNNING)
    def getIsLocalized(self, req):
        if self.anchordetected:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    
    def doMoveTo(self, req):
        # TODO: the move to service needs the following functionality
        # 1) During approach verify that the object is where we found it 
        # 2) If the object is gone, tell the map to whipe that area and set it to free
        # 3) If the object is reclassified, change the object class and return failure
        
        # 1)
        # This is too convoluted to implement as a single method. It needs to be expanded into a node
        if self.targetHasBecomeInvalid:
            self.targetHasBecomeInvalid = False
            return Moveto(FAILURE)
        req = Moveto()
        req.goal = self.targetToy.poseStamped

        res = self.pathPlanner_proxy(req)
        STATUS = res.status
        return MovetoResponse(STATUS)
    
    def __putObject(self, pose: PoseStamped, id: int):
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
        else:
            raise Exception("Invalid object ID: " % str(id))
        
        name = self.id2Object[id] + "_" + str(objectType.count)
        objectType.count += 1
        object = objectType(pose=pose, name=name, id=id)
        self.objects[name] = object
        self.toys[name] = object
        correctDict[name] = object
        

    def __putBox(self, object: Box, replace: bool = False, object_to_replace: str = None):
        if object.hasArucoMarker:
            if replace:
                del self.objects[object_to_replace]
            self.objects[object.name] = object
            self.boxes[object.name] = object
        else:
            Box.count += 1
            self.objects[object.name] = object
            self.boxes[object.name] = object

    def __doStoreAllDetectedObjects(self, msg: objectPoseStampedLst):
        for pose, id in zip(msg.PoseStamped, msg.object_class):
            print(id, self.object2Id[id])
            id = self.object2Id[id]
            if id < 8:
                self.__putInDictsIfNotAlreadyIn(pose,id)
            elif id == 8:
                boxObject = Box(pose, self.id2Object[id] + str(Box.count), id)
                self.__putBoxInDict(boxObject)
    def __getWithinRange(self, a: Point, b: Point) -> bool:
        if abs(a.x - b.x) < self.xThreshold and abs(a.y - b.y) < self.yThreshold:
            return True
        return False
    def __doStoreAllBoxesWAruco(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id == 500:
                self.anchordetected = True
                continue
            boxObject = Box(marker.pose.pose, self.arucoId2Box[marker.id], marker.id)
            boxObject.hasArucoMarker = True
            self.__putBoxInDict(boxObject)

    def __putBoxInDict(self, boxObject: Box):
        for object_name in self.objects:
            objectPos = self.objects[object_name].poseStamped.pose.position
            boxPos = boxObject.poseStamped.pose.position
            if self.__getWithinRange(objectPos, boxPos):
                if not object.hasArucoMarker and boxObject.hasArucoMarker:
                    replace = True
                    self.putBox(boxObject,replace,object_name)
                    return
        self.__putBox(boxObject)
            
    
    def __putInDictsIfNotAlreadyIn(self, pose: PoseStamped, id: int):
        for key in self.objects:
            object = self.objects[key]
            if self.__getWithinRange(a=object.poseStamped.pose.position, b=pose.pose.position):
                #if self.movingToTargetToy and object.name==self.targetToy.name and self.targetToy.id != id:
                    # Grumpy is looking at a toy, lets call it the observed toy
                    # The observed toy is where the target toy is supposed to be
                    # However the observed toy is not of the same id as the target toy
                    # A missclassification has occured
                #    self.targetHasBecomeInvalid = True
                return
        self.__putObject(pose, id)


if __name__ == "__main__":

    rospy.init_node("memory")
    m = Memory()
    rospy.spin()    
