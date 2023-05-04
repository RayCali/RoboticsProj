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
from msg_srv_pkg.srv import Request, RequestResponse, RequestRequest
from visualization_msgs.msg import Marker
from config import SUCCESS, RUNNING, FAILURE
# https://stackoverflow.com/questions/42660670/collapse-all-methods-in-visual-studio-code
class Memory:
    def __init__(self):
        self.objects: Dict[Movable]= {}
        self.plushies: Dict[Plushie] = {}
        self.cubes: Dict[Cube] = {}
        self.balls: Dict[Ball] = {}
        self.toys: Dict[Toy] = {}
        self.toys_buffer = []
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

        self.anchor_sub = rospy.Subscriber("/aruco_500/aruco/markers", MarkerArray, self.doSetAnchorAsDetected) 
        self.detection_sub = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.doStoreAllDetectedObjects)
        self.aruco_sub = rospy.Subscriber("/aruco_all/aruco/markers", MarkerArray, self.doStoreAllBoxesWAruco)

        # self.moveto_srv = rospy.Service("/moveto", Moveto, self.doMoveTo)
        self.isLocalized_srv = rospy.Service("/isLocalized", Request, self.getIsLocalized)
        self.doLocalize_srv = rospy.Service("/doLocalize", Request, self.doLocalize)

        self.isnotpair_srv = rospy.Service("/notpair", Request, self.getNotPair)

        # self.ispicked_srv = rospy.Service("/ispicked", Request, self.getIsPicked)
        # self.isInFrontToy_srv = rospy.Service("/isInFrontToy", Request, self.getIsInFrontToy)
        # self.pathPlanner_proxy = rospy.ServiceProxy("/pathPlanner", Moveto)

        self.isFound_srv = rospy.Service("/isFound", Request, self.getIsFound)
        self.toyPub = rospy.Publisher("/toyPoseMap", PoseStamped, queue_size=10)

        # self.isExplored_srv = rospy.Service("/isExplored", Request, self.getIsExplored)
        
        self.movingToTargetToy = False
        self.targetToy: Toy = None
        self.targetHasBecomeInvalid = False
        self.xThreshold = 0.10
        self.yThreshold = 0.10
        # self.doPick_srv = rospy.Service("pickup", Pick, self.doInformOfPick)
        self.anchordetected = False
    
    def getIsFound(self, req: RequestRequest):
        if len(self.toys) > 0:
            print("Found a toy")
            self.toyPub.publish(self.toys[list(self.toys.keys())[0]].poseStamped)
            return RequestResponse(SUCCESS)
        print("NO TOY!")
        return RequestResponse(FAILURE)
    
    # def getIsExplored(self, req: RequestRequest):
    #     if self.getNotPair == SUCCESS:
    #         print("NO PAIR!")
    #         print("START EXPLORING")
    #         return RequestResponse(SUCCESS)
    #     print("PAIR FOUND!")
    #     print("STOP EXPLORING")
    #     return RequestResponse(FAILURE)
    
    # def doInformOfPick(self, req):
    #     if self.alreadyPickingUpAnObjectSinceBefore:
    #         return Pick(RUNNING)
    #     else:
    #         self.alreadyPickingUpAnObjectSinceBefore = True
    def doMoveToGoal(self, req: RequestRequest):
        return RequestResponse(RUNNING)    
    def getIsInFrontToy(self, req: RequestRequest):
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
    
    # def getIsPicked(self, req):
    #     return RequestResponse(FAILURE)
    def getNotPair(self, req: RequestRequest):
        # TODO: check if there is a valid box-object pair in the memory and return FAILURE if there IS and SUCCESS if ther IS NOT.
        # we have a pair if
        # 1) the box object has an aruco marker so we can identify which box it is
        # 2) the dictionary of the object class that the box belongs to is not empty
        STATUS = SUCCESS
        for key in self.boxes:
            box = self.boxes[key]
            if box.hasArucoMarker:
                if box.name == "Box_Plushies":
                    if len(self.plushies) > 0:
                        STATUS = FAILURE
                        self.targetToy = self.plushies[self.plushies.keys()[0]]
                elif box.name == "Box_Balls":
                    if len(self.balls) > 0:
                        STATUS = FAILURE
                        self.targetToy = self.balls[self.balls.keys()[0]]

                elif box.name == "Box_Cubes":
                    if len(self.cubes) > 0:
                        STATUS = FAILURE
                        self.targetToy = self.cubes[self.cubes.keys()[0]]
        
        return RequestResponse(STATUS)
    
    def doLocalize(self, req: RequestRequest):
        if not self.anchordetected:
            return RequestResponse(RUNNING)
        return RequestResponse(SUCCESS)
    def getIsLocalized(self, req: RequestRequest):
        if self.anchordetected:
            print("anchor detected")
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    def doSetAnchorAsDetected(self, msg: MarkerArray):
        self.anchordetected = True
    
    # def doMoveTo(self, req):
    #     # TODO: the move to service needs the following functionality
    #     # 1) During approach verify that the object is where we found it 
    #     # 2) If the object is gone, tell the map to whipe that area and set it to free
    #     # 3) If the object is reclassified, change the object class and return failure
        
    #     if self.targetHasBecomeInvalid:
    #         self.targetHasBecomeInvalid = False
    #         return Moveto(FAILURE)
    #     req = Moveto()
    #     req.goal = self.targetToy.poseStamped

    #     res = self.pathPlanner_proxy(req)
    #     STATUS = res.status
    #     if STATUS == FAILURE:
    #         # If status returned failure then the goal is unreachable. We need to select a new goal
    #         self.targetToy.unreachable = True
    #         self.targetToy = None
    #     return MovetoResponse(STATUS)
    
    def putObject(self, pose: PoseStamped, id: int):
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

    def putObjectinBuffer(self, pose: PoseStamped, id: int):
        count = 0
        for toy_pose, toy_id in self.toys_buffer:
            if self.getWithinRange(toy_pose.pose.position, pose.pose.position):
                if toy_id == id:
                    count += 1
        if count < 15:
            self.toys_buffer.append((pose,id))
        if count > 14:
            self.putObject(pose,id)
            self.toys_buffer = [(pose_keep, id_keep) for pose_keep, id_keep in self.toys_buffer if id_keep != id and not self.getWithinRange(pose_keep.pose.position, pose.pose.position) and (rospy.Time.now().secs - pose_keep.header.stamp.secs) < 5]


        

    def putBox(self, object: Box, replace: bool = False, object_to_replace: str = None):
        if object.hasArucoMarker:
            if replace:
                del self.objects[object_to_replace]
            self.objects[object.name] = object
            self.boxes[object.name] = object
        else:
            Box.count += 1
            self.objects[object.name] = object
            self.boxes[object.name] = object

    def doStoreAllDetectedObjects(self, msg: objectPoseStampedLst):
        for pose, id in zip(msg.PoseStamped, msg.object_class):
            id = self.object2Id[id]
            if id < 8:

                self.putInDictsIfNotAlreadyIn(pose,id)
            elif id == 8:
                boxObject = Box(pose, self.id2Object[id] + str(Box.count), id)
                self.putBoxInDict(boxObject)
    def getWithinRange(self, a: Point, b: Point) -> bool:
        if abs(a.x - b.x) < self.xThreshold and abs(a.y - b.y) < self.yThreshold:
            return True
        return False
    def doStoreAllBoxesWAruco(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id == 500:
                self.anchordetected = True
                continue
            boxObject = Box(marker.pose.pose, self.arucoId2Box[marker.id], marker.id)
            boxObject.hasArucoMarker = True
            self.putBoxInDict(boxObject)

    def putBoxInDict(self, boxObject: Box):
        for object_name in self.objects:
            objectPos = self.objects[object_name].poseStamped.pose.position
            boxPos = boxObject.poseStamped.pose.position
            if self.getWithinRange(objectPos, boxPos):
                if not object.hasArucoMarker and boxObject.hasArucoMarker:
                    replace = True
                    self.putBox(boxObject,replace,object_name)
                    return
        self.putBox(boxObject)
            
    
    def putInDictsIfNotAlreadyIn(self, pose: PoseStamped, id: int):
        for key in self.objects:
            object = self.objects[key]
            if self.getWithinRange(a=object.poseStamped.pose.position, b=pose.pose.position):
                #if self.movingToTargetToy and object.name==self.targetToy.name and self.targetToy.id != id:
                    # Grumpy is looking at a toy, lets call it the observed toy
                    # The observed toy is where the target toy is supposed to be
                    # However the observed toy is not of the same id as the target toy
                    # A missclassification has occured
                #    self.targetHasBecomeInvalid = True
                return
            
        self.putObjectinBuffer(pose, id)
    

if __name__ == "__main__":

    rospy.init_node("memory")
    m = Memory()
    rospy.spin()    
