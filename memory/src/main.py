#!/usr/bin/python3
import rospy
import numpy as np
from numpy import dot, array
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Point,TransformStamped
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
from std_msgs.msg import Bool
from config import SUCCESS, RUNNING, FAILURE
import math
from playsound import playsound
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
           6 : "Cube",
           7 : "Ball",
           8 : "Box",
        }
        self.object2Id = {
            "Binky"  : 0,
            "Hugo"   : 1,
            "Slush"  : 2,
            "Muddles": 3,
            "Kiki"   : 4,
            "Oakie"  : 5,
            "Cube"   : 6,
            "Ball"   : 7,
            "Box"    : 8
        }
        self.arucoId2Box = {
            2 : "Box_Plushies",
            3 : "Box_Balls",
            1 : "Box_Cubes",
            500 : "Anchor"
        }

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.anchor_sub     = rospy.Subscriber("/aruco_500/aruco/markers", MarkerArray, self.doSetAnchorAsDetected) 
        self.detection_sub  = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.doStoreAllDetectedObjects)
        self.aruco_sub      = rospy.Subscriber("/aruco_all/aruco/markers", MarkerArray, self.doStoreAllBoxesWAruco, queue_size=1)

        self.isLocalized_srv    = rospy.Service("/srv/isLocalized/memory/brain", Request, self.getIsLocalized)
        self.doLocalize_srv     = rospy.Service("/srv/doLocalize/memory/brain", Request, self.doLocalize)
        self.isnotpair_srv      = rospy.Service("/srv/isNotPair/memory/brain", Request, self.getNotPair)
        self.isAtToy_srv        = rospy.Service('/srv/isAtToy/memory/brain', Request, self.getIsAtToy)
        self.move2Toy_srv       = rospy.Service("/srv/doMoveAlongPathToyLocal/memory/brain", Request, self.doMoveAlongPathToyLocal)
        self.isPlanned_srv      = rospy.Service("/srv/isPlanned/memory/brain", Request, self.getIsPlanned)
        
        self.isPicked_srv       = rospy.Service("/srv/isPicked/memory/brain", Request, self.getIsPicked)
        self.doPickup_srv       = rospy.Service("/srv/doPickToy/memory/brain", Request, self.doPickup)
        self.doPlanPathToy_srv  = rospy.Service("/srv/doPlanpathToy/memory/brain", Request, self.doPlanPathToy)
        self.isAtBox_srv        = rospy.Service("/srv/isAtBox/memory/brain", Request, self.getIsAtBox)
        self.isPlannedBox_srv   = rospy.Service("/srv/isPlannedBox/memory/brain", Request, self.getIsPlannedBox)
        self.doPlanPathBox_srv  = rospy.Service("/srv/doPlanPathBox/memory/brain", Request, self.doPlanPathBox)
        self.move2Box_srv       = rospy.Service("/srv/doMoveAlongPathBoxLocal/memory/brain", Request, self.doMoveAlongPathBoxLocal)
        self.moveback_srv       = rospy.Service("/srv/doMoveBack/memory/brain", Request, self.doMoveBack)
        self.hasMovedBack_srv = rospy.ServiceProxy('/srv/hasMovedBack/memory/brain', Request, self.getHasMovedBack)
        self.isPlaced_srv       = rospy.Service("/srv/isPlaced/memory/brain", Request, self.getIsPlaced)
        self.doPlace_srv        = rospy.Service("/srv/doPlace/memory/brain", Request, self.doPlace)


        self.isPlanExplored     = rospy.Service("/srv/isGoalSelected/memory/brain", Request, self.getIsSlected)
        self.mostValue_srv      = rospy.Service("/srv/doSelectExplorationGoal/memory/brain", Request, self.doSelectExplorationGoal)
        self.isExplorePlaned_srv= rospy.Service("/srv/isExplorationPathPlanned/memory/brain", Request, self.getIsExplorationPathPlanned)
        self.move2Explore_srv   = rospy.Service("/srv/doMoveAlongPathGlobal/memory/brain", Request, self.doMoveAlongPathGlobal)
        self.explore_srv        = rospy.Service("/srv/doPlanExplorationPath/memory/brain", Request, self.doPlanpathExplore)
        self.pathplanpub = rospy.Publisher("/goalTarget", objectPoseStampedLst, queue_size=10)
        self.reset_behaviour_pub = rospy.Publisher("/RESET", Bool, queue_size=10)
        self.goal_name = "lmao"
                        
        
        self.targetBox: Box= None
        self.targetToy: Toy = None
        self.xThreshold = 0.10
        self.yThreshold = 0.10
        self.anchordetected = False
        self.isSelected = False
        self.pathToExplorationGoalPlanned = False
        self.hasMovedBack = False

    
    def getIsExplorationPathPlanned(self, req: RequestRequest):
        if self.pathToExplorationGoalPlanned:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    def doMoveAlongPathGlobal(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doMoveAlongPathGlobal/path_follower_global/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.isSelected = False
            self.pathToExplorationGoalPlanned = False
        if res.success == FAILURE:
            self.isSelected = False
            self.pathToExplorationGoalPlanned = False
        return res
    

    def getIsSlected(self, req: RequestRequest):
        if self.isSelected:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)

    def doSelectExplorationGoal(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doSelectExplorationGoal/mapping_and_planning/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.isSelected = True
        return res        

    def isExplorationPathPlanned(self, req: RequestRequest):
        if self.pathToExplorationGoalPlanned:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)

    def doPlanpathExplore(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doPlanpath/mapping_and_planning/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.pathToExplorationGoalPlanned = True
        return res
    

    def getIsPlaced(self, req: RequestRequest):
        if self.targetToy.inBox:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    
    def doPlace(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doPlace/pickup/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.targetToy.inBox = True
            self.targetToy.isPicked = False
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)


    def doMoveAlongPathBoxLocal(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doMoveAlongPathBoxLocal/path_follower_local/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.targetBox.atBox = True
            self.targetBox.isPlanned = False
            self.targetToy.atToy = False
        if res.success == FAILURE:
            self.targetBox.isPlanned = False
        return res
    def doMoveBack(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doMoveBack/point_follower_aruco/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.hasMovedBack = True
        else:
            self.hasMovedBack = False
        return res
    def getHasMovedBack(self, req: RequestRequest):
        if self.hasMovedBack:
            return RequestResponse(SUCCESS)
        else:
            return RequestResponse(FAILURE)    
    def getIsPlannedBox(self, req: RequestRequest):
        pose = self.targetBox.poseStamped
        if self.targetBox.isPlanned:
            return RequestResponse(SUCCESS)
        # This is when the path planner gets the pose of the box
        object_poses = objectPoseStampedLst()
        name = self.targetBox.name
        ps = self.targetBox.poseStamped
        object_poses.PoseStamped.append(ps)
        object_poses.object_class.append(name)
        self.pathplanpub.publish(object_poses)
        rospy.sleep(1)
        return RequestResponse(FAILURE)
    

    def getIsAtBox(self, req: RequestRequest):
        if self.targetBox.atBox:
            return RequestResponse(SUCCESS)
        to_be_published = objectPoseStampedLst()
        to_be_published.PoseStamped.append(self.targetBox.poseStamped)
        to_be_published.object_class.append(self.targetBox.name)
        self.pathplanpub.publish(to_be_published)
        return RequestResponse(FAILURE)
    

    def doPlanPathBox(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doPlanpath/mapping_and_planning/memory", Request)
        res: RequestResponse = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.targetToy.isPlanned = True
        return res

    
    def doPickup(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doPickToy/pickup/memory", Request)
        res = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.targetToy.isPicked = True
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    def getIsPicked(self, req: RequestRequest):
        if self.targetToy.isPicked:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    def getIsPlanned(self, req: RequestRequest):
        if self.targetToy.isPlanned:
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    
    def doPlanPathToy(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doPlanpath/mapping_and_planning/memory", Request)
        res: RequestResponse = proxy(RequestRequest())
        if res.success == SUCCESS:
            self.targetToy.isPlanned = True
        return res
    
    

    def doMoveAlongPathToyLocal(self, req: RequestRequest):
        proxy = rospy.ServiceProxy("/srv/doMoveAlongPathToyLocal/path_follower/memory", Request)
        
        res: RequestResponse = proxy(RequestRequest())
        print("Response", res)
        if res.success == SUCCESS:
            self.targetToy.atToy = True
            self.targetToy.isPlanned = False
        if res.success == FAILURE:
            self.targetToy.isPlanned = False
        return res
    def getIsAtToy(self, req: RequestRequest):
        if self.targetToy.atToy:
            return RequestResponse(SUCCESS)
        to_be_published = objectPoseStampedLst()
        to_be_published.PoseStamped.append(self.targetToy.poseStamped)
        to_be_published.object_class.append(self.targetToy.name)
        self.pathplanpub.publish(to_be_published)
        return RequestResponse(FAILURE)
   
    
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
        print("getNotPair. Len(boxes):", len(self.boxes))
        for key in self.boxes:
            box: Box = self.boxes[key]
            print("Box: ", box.name)
            if box.hasArucoMarker:
                boxPose = objectPoseStampedLst()
                objectPose = objectPoseStampedLst()
                print(box.name)

                    #     self.arucoId2Box = {
                    #     2 : "Box_Plushies",
                    #     3 : "Box_Balls",
                    #     1 : "Box_Cubes",
                    #     500 : "Anchor"
                    # }
                pickable_plushies = [self.plushies[key] for key in list(self.plushies.keys()) if not self.plushies[key].inBox]
                print("pickable plushies: ", len(pickable_plushies))
                
                pickable_balls = [self.balls[key] for key in list(self.balls.keys()) if not self.balls[key].inBox]
                print("pickable balls: ", len(pickable_balls))
                
                
                pickable_cubes = [self.cubes[key] for key in list(self.cubes.keys()) if not self.cubes[key].inBox]
                print("pickable cubes: ", len(pickable_cubes))                    
                
                if box.name == "Box2":
                    if len(pickable_plushies) > 0:
                        self.targetBox = box
                        self.targetToy = pickable_plushies[0]
                        boxPose.PoseStamped.append(box.poseStamped)
                        boxPose.object_class.append(box.name)
                        objectPose.PoseStamped.append(self.targetToy.poseStamped)
                        objectPose.object_class.append(self.targetToy.name)
                        return RequestResponse(FAILURE)

                        
                elif box.name == "Box3":
                    if len(pickable_balls) > 0:
                        self.targetBox = box
                        self.targetToy = pickable_balls[0]
                        boxPose.PoseStamped.append(box.poseStamped)
                        boxPose.object_class.append(box.name)
                        objectPose.PoseStamped.append(self.targetToy.poseStamped)
                        objectPose.object_class.append(self.targetToy.name)
                        return RequestResponse(FAILURE)

                elif box.name == "Box1":
                    if len(pickable_cubes) > 0:
                        self.targetBox = box
                        self.targetToy = pickable_cubes[0]
                        boxPose.PoseStamped.append(box.poseStamped)
                        boxPose.object_class.append(box.name)
                        objectPose.PoseStamped.append(self.targetToy.poseStamped)
                        objectPose.object_class.append(self.targetToy.name)
                        return RequestResponse(FAILURE)

                rospy.loginfo("No PAIR status: {}".format(SUCCESS))

        return RequestResponse(SUCCESS)
    
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

        self.playingSound(id)  

    def playingSound(self, id: int):
        playsound('/home/robot/Downloads' + str(self.id2Object[id]) + ".mp3")

    def putObjectinBuffer(self, pose: PoseStamped, id: int):
        count = 0
        for toy_pose, toy_id in self.toys_buffer:
            if self.getWithinRange(toy_pose.pose.position, pose.pose.position):
                if toy_id == id:
                    count += 1
        print("count: ", count)
        if count < 10:
            self.toys_buffer.append((pose,id))
        if count > 9:
            self.putObject(pose,id)
            self.toys_buffer = [(pose_keep, id_keep) for pose_keep, id_keep in self.toys_buffer if id_keep != id and not self.getWithinRange(pose_keep.pose.position, pose.pose.position) and (rospy.Time.now().secs - pose_keep.header.stamp.secs) < 5]


        

    def putBox(self, object: Box, replace: bool = False, object_to_replace: str = None):
        print("putting box: ", object.name)
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
            # elif id == 8:
            #     boxObject = Box(pose, self.id2Object[id] + str(Box.count), id)
            #     self.putBoxInDict(boxObject)
    def getWithinRange(self, a: Point, b: Point) -> bool:
        if abs(a.x - b.x) < self.xThreshold and abs(a.y - b.y) < self.yThreshold:
            return True
        return False

    def doStoreAllBoxesWAruco(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id == 500:
                self.anchordetected = True
                continue
            self.putArucoMarkerIntoDictionaryAsABoxIfNotAlreadyDetected(marker)
    def putArucoMarkerIntoDictionaryAsABoxIfNotAlreadyDetected(self, marker: Marker):
        boxPose_cam = PoseStamped()
        boxPose_cam.pose = marker.pose.pose
        transform = self.tf_buffer.lookup_transform('map', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
        boxPose_map = tf2_geometry_msgs.do_transform_pose(boxPose_cam, transform)
        t = TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "Box" + str(marker.id)
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = boxPose_map.pose.position.x
        t.transform.translation.y = boxPose_map.pose.position.y
        t.transform.translation.z = 0
        anglelist = [boxPose_map.pose.orientation.x, boxPose_map.pose.orientation.y, boxPose_map.pose.orientation.z, boxPose_map.pose.orientation.w]
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(anglelist)
        roll = roll - math.pi/2
        yaw = yaw - math.pi/2
        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        for key in self.objects:
            object = self.objects[key]
            if self.getWithinRange(a=object.poseStamped.pose.position, b=boxPose_map.pose.position):
                return        
        self.br.sendTransform(t)
        boxPose_aruco = PoseStamped()
        boxPose_aruco.header.frame_id = "Box" + str(marker.id)
        boxPose_aruco.pose.position.x = 0.2
        boxPose_aruco.pose.position.y = 0
        boxPose_aruco.pose.position.z = 0
        boxPose_aruco.pose.orientation.x = 0
        boxPose_aruco.pose.orientation.y = 0
        boxPose_aruco.pose.orientation.z = 0
        boxPose_aruco.pose.orientation.w = 1
        transform = self.tf_buffer.lookup_transform('map', 'Box' + str(marker.id), rospy.Time(0), rospy.Duration(1.0))
        boxPose_map = tf2_geometry_msgs.do_transform_pose(boxPose_aruco, transform)
        boxObject = Box(boxPose_map, "Box" + str(marker.id), marker.id)
        boxObject.hasArucoMarker = True
        self.putBoxInDict(boxObject)

    def putBoxInDict(self, boxObject: Box):
        for object_name in self.objects:
            objectPos = self.objects[object_name].poseStamped.pose.position
            boxPos = boxObject.poseStamped.pose.position
            if self.getWithinRange(objectPos, boxPos):
                if boxObject.hasArucoMarker:
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
