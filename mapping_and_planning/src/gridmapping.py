#!/usr/bin/python3

import rospy
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped
from msg_srv_pkg.srv import Request, RequestResponse, RequestRequest
from global_explorer import getMostValuedCell 
from std_msgs.msg import Int64
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
import math



from geometry_msgs.msg import PoseStamped
from msg_srv_pkg.msg import objectPoseStampedLst
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
from typing import List
import matplotlib.path as mpltPath
from config import SUCCESS, RUNNING, FAILURE
from visualization_msgs.msg import Marker
from std_msgs.msg import Int64
from nav_msgs.msg import Path
from std_msgs.msg import Float64
SUCCESS, RUNNING, FAILURE = 1, 0, -1

class Map():
    def __init__(self, plot=False, width=1000, height=1000, resolution=0.1):
        
        self.anchordetected = False
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.no_collision_srv = rospy.Service('/srv/no_collision/mapping_and_planning/path_follower', Request, self.__nocollision)
        self.lineitup_sub = rospy.Subscriber("/boundaries", Marker, self.__lineitup)
        self.save_sub   = rospy.Subscriber('/rewired', Path, self.__savepath)
        self.nodenr_sub   = rospy.Subscriber('/path_follower/node', Float64, self.__savenode)
        self.stop_explore_pub = rospy.Publisher("/stopexploring", Int64, queue_size=1)
        width = int(width/resolution)
        height = int(height/resolution)
        self.ones = False
        self.matrix = np.zeros((width, height), dtype=np.int8)
        self.start_explore_srv = rospy.Service("/srv/doExplore/mapping_and_planning/brain", Request, self.__doStartExploreCallback)
        self.stop_explore_srv = rospy.Service("/srv/stopExplore/mapping_and_planning/brain", Request, self.__stopExploreCallback)
        self.explore_pub= rospy.Publisher("/explore", Int64, queue_size=1)
        self.explore_sub = rospy.Subscriber("/explore", Int64, self.__exploreCallback)
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "map"
        self.grid.info.resolution = resolution
        self.grid.info.width = width
        self.grid.info.height = height
        self.grid_naming_convention = {
            "unknown": 0,
            "free": 1,
            "occupied": 2,
            "toy": 3,
            "box": 4,
            "outside of workspace": 5,
            "on the line": 6
        }
        
        self.grid.info.origin = Pose(Point(-2.0, -6.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)) #This is the real-world pose of the
        self.grid.data = None
        self.grid_sub = rospy.Subscriber("/scan", LaserScan, self.__doScanCallback, queue_size=1)
        self.grid_pub = rospy.Publisher("/topic", OccupancyGrid, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/mostValuedCell", PoseStamped, queue_size=10)
        self.received_mostvaluedcell = False
        self.startExplore_STATE = FAILURE
        self.stopExplore_STATE = FAILURE

        self.start_explore = rospy.Publisher("/start_explore", Int64, queue_size=1)
        self.workspace_sub = rospy.Subscriber("/boundaries", Marker, self.__doWorkspaceCallback)
        self.detection_sub = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.__doObjectCallback)
        self.class_dictionary = {
            "Binky" : 3,
            "Hugo" : 3,
            "Slush" : 3,
            "Muddles": 3,
            "Kiki": 3,
            "Oakie": 3,
            "cube": 3,
            "ball": 3,
            "box": 4,

        }
    # def __doStartExploreCallback(self, req: RequestRequest):
    #     ts: TransformStamped = getMostValuedCell(self.matrix, int(self.grid.info.width), int(self.grid.info.height), float(self.grid.info.resolution), (self.grid.info.origin.position.x, self.grid.info.origin.position.y))
    #     ps: PoseStamped = PoseStamped()
    #     stopExplore = Int64()
    #     stopExplore.data = 0
    #     self.stop_explore_pub.publish(stopExplore)
    #     ps.header = ts.header
    #     ps.pose.position.x = ts.transform.translation.x
    #     ps.pose.position.y = ts.transform.translation.y
    #     self.goal_pub.publish(ps)
    #     return RequestResponse(SUCCESS)
    
    def __exploreCallback(self, msg: Int64):
        ts: TransformStamped = getMostValuedCell(self.matrix, int(self.grid.info.width), int(self.grid.info.height), float(self.grid.info.resolution), (self.grid.info.origin.position.x, self.grid.info.origin.position.y))
        ps: PoseStamped = PoseStamped()
        ps.header = ts.header
        ps.pose.position.x = ts.transform.translation.x
        ps.pose.position.y = ts.transform.translation.y
        self.goal_pub.publish(ps)
        self.startExplore_STATE = SUCCESS
        return
    def __doStartExploreCallback(self, req: RequestRequest):
        if self.received_mostvaluedcell:
            return RequestResponse(SUCCESS)
        if not self.running:
            self.running = True
            self.explore_pub.publish(Int64())
            return RequestResponse(RUNNING)
        if self.running:
            if self.startExplore_STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.startExplore_STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)
            if self.startExplore_STATE == SUCCESS:
                self.running = False
                self.received_mostvaluedcell = True
                return RequestResponse(SUCCESS)
        
    
    def __stopExploreCallback(self, req: RequestRequest):
        if not self.running:
            return RequestResponse(RUNNING)
        if self.running:
            if self.stopExplore_STATE == RUNNING:
                stopExplore = Int64()
                stopExplore.data = 1
                self.stop_explore_pub.publish(stopExplore)
                self.stopExplore_STATE = SUCCESS
                return RequestResponse(RUNNING)
            if self.stopExplore_STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)
            if self.stopExplore_STATE == SUCCESS:
                self.running = False
                return RequestResponse(SUCCESS)


    def __getOccupancyGridObject(self) -> OccupancyGrid:
        self.grid.data = self.matrix.flatten()
        self.grid.data[self.grid.data == 0] = 20
        self.grid.data[self.grid.data == 1] = 0
        self.grid.data[self.grid.data == 2] = 95
        self.grid.data[self.grid.data == 3] = 50
        self.grid.data[self.grid.data == 4] = 75
        self.grid.data[self.grid.data == 5] = 100
        self.grid.data[self.grid.data == 6] = 40
        

        return self.grid
    
    def doPublish(self):
        rate = rospy.Rate(10.0)
        og = self.__getOccupancyGridObject()
        # rospy.loginfo(og)
        self.grid_pub.publish(og)
        rate.sleep()

    def __getProbabilityFromMatrixValue(self, x: int) -> int:
        if x == 0:      
            return 20   # unknown space
        if x == 1:      
            return 0    # free space
        if x == 2:
            return 95  # black(obstacles)
        if x == 3:
            return 50   # toy
        if x == 4:
            return 75   # box
        if x== 5:
            return 100  # out
        else:
            raise Exception("None value in matrix")


    def __doScanCallback(self, msg: LaserScan):
        global latestscan
        latestscan = msg
        latestupdate = rospy.Time(0)
        laserlist = []
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", latestupdate, rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)

        anglelist = tf_conversions.transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        for i in range(len(msg.ranges)):
            objectorfree = 2
            laserlist.append(msg.ranges[i])
            if laserlist[i] > 2:
                laserlist[i] = 2
                objectorfree=1
            if laserlist[i] <= 2:
                
                x = transform.transform.translation.x + laserlist[i] * np.cos(msg.angle_min + i * msg.angle_increment + anglelist[2])
                y = transform.transform.translation.y + laserlist[i] * np.sin(msg.angle_min + i * msg.angle_increment + anglelist[2])
                
                x_ind = int((x - self.grid.info.origin.position.x) / self.grid.info.resolution)
                y_ind = int((y - self.grid.info.origin.position.y) / self.grid.info.resolution)
                if x_ind>self.grid.info.height or y_ind>self.grid.info.width:
                    print("FUCKUP!NOTGOOD!VERY BAD!!!")
                    exit()
                self.__doDrawFreespace(
                    r=laserlist[i], 
                    x0 = transform.transform.translation.x,
                    y0 = transform.transform.translation.y,
                    x1 = x,
                    y1 = y,
                    x_1_ind=x_ind,
                    y_1_ind=y_ind 
                    )
                try:
                    if self.matrix[y_ind, x_ind] != 5:
                        self.matrix[y_ind, x_ind] = objectorfree
                except IndexError:
                    print("Outside grid")
                    exit()
                
                # print(x,y)
                # print(x_ind, y_ind)
                # print(self.matrix.shape)
                # print(self.matrix[y_ind, x_ind])
                # print()

                
            self.grid.header.stamp = rospy.Time.now()
        
            

    def __doDrawFreespace(self, r:float, x0: float, y0: float, x1: float, y1:float, x_1_ind:int, y_1_ind:int):
        R = int(r / self.grid.info.resolution)
        indices = []
        dx = x1 - x0
        dy = y1 - y0
        for i in range(0, R, 1):
            xi = x0 + dx * i / R 
            yi = y0 + dy * i / R
            x_i_ind = int((xi - self.grid.info.origin.position.x) / self.grid.info.resolution)
            y_i_ind = int((yi - self.grid.info.origin.position.y) / self.grid.info.resolution)
            if x_i_ind != x_1_ind and y_i_ind != y_1_ind:
                indices.append((x_i_ind, y_i_ind))
        # print()
        # print()
        for x,y in indices:
            self.__doCheckForFreeSpaceAndInsert(x, y, x_1_ind, y_1_ind)
        
    
    def __doCheckForFreeSpaceAndInsert(self, x:int, y:int, xmax:int, ymax:int):
        cell = self.matrix[y,x]
        if cell == 0 or cell == 1 or cell == 2: #painting over, UNK,OBS and FRE to FRE
            self.matrix[y,x] = 1
        elif cell == 3 or cell == 4: # TOY or BOX found between us and the obstacle
            pass
        elif cell == 5 or cell == 6: # This area is outside of the workspace
            pass
        else:
            rospy.loginfo("  INVALID VALUE FOUND IN THE GRID MATRIX")
            raise Exception("INVALID VALUE FOUND IN THE GRID MATRIX") 
        pass
        

    def __doDrawBox(self):
        boxSize = int(min(self.grid.info.width, self.grid.info.height) / 10) 
        lower = int(min(self.grid.info.width, self.grid.info.height) / 10)
        
        for i in range(lower, lower + boxSize, 1):
            for ii in range(lower, lower + boxSize, 1):
                self.matrix[i, ii] = 1

        lower = int(min(self.grid.info.width, self.grid.info.height) / 2)
        transform = self.tf_buffer.lookup_transform("map", "arucomap", rospy.Time(0), rospy.Duration(2))
        offset = 0
        x = transform.transform.translation.x + 1 + offset - self.grid.info.origin.position.x
        y = transform.transform.translation.y - self.grid.info.origin.position.y
        x = int(x/self.grid.info.resolution)
        y = int(y/self.grid.info.resolution)
        
        for i in range(-10,10,1):
            self.matrix[y+i,x] = 2
    
    def __nocollision(self,req:RequestRequest):
        global latestscan, path, nodenr
        for i in range (len(latestscan.ranges)):
            if latestscan.ranges[i] < 0.5:
                if latestscan.angle_min + i * latestscan.angle_increment < 0.5 and latestscan.angle_min + i * latestscan.angle_increment > -0.5:
                    return RequestResponse(FAILURE)
        # position_with_toys = np.where(self.matrix == 3)
        # currentnode = path.poses[int(nodenr)]
        # base_link = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2))
        # for i in range(len(position_with_toys[0])):
        #     p1 = np.array([currentnode.pose.position.x,currentnode.pose.position.y])
        #     p2 = np.array([base_link.transform.translation.x,base_link.transform.translation.y])
        #     p3 = np.array([position_with_toys[1][i]*self.grid.info.resolution+self.grid.info.origin.position.x,position_with_toys[0][i]*self.grid.info.resolution+self.grid.info.origin.position.y])
        #     d=np.abs(np.cross(p2-p1,p3-p1))/np.linalg.norm(p2-p1)
        #     if d < 0.5: #will depend on covariance
        #         return RequestResponse(FAILURE)
        return RequestResponse(SUCCESS)
    def __savenode(self,msg):
        global nodenr
        nodenr = msg.data
    def __savepath(self,msg):
        global path
        path = msg

    def point_inside_polygon(self,x,y,poly):
        n = len(poly)
        inside =False
        p1x,p1y = poly[0].x,poly[0].y
        for i in range(n+1):
            p2x,p2y = poly[i % n].x,poly[i % n].y
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y
        return inside
    def __lineitup(self, msg: Marker):
        rospy.loginfo("LINE IT UP")
        poly = msg
        pointlist = [[]]
        rospy.loginfo(poly)
        margin = 0.3
        trans = self.tf_buffer.lookup_transform("map", "arucomap", rospy.Time(0))
        for i in range(len(poly.points)):
            apoint = PoseStamped()
            apoint.header.frame_id = "arucomap"
            apoint.pose.position.x = poly.points[i].x
            apoint.pose.position.y = poly.points[i].y
            apoint.pose.position.z =0
            apoint_new = tf2_geometry_msgs.do_transform_pose(apoint,trans)
            poly.points[i].x = apoint_new.pose.position.x
            poly.points[i].y = apoint_new.pose.position.y
        prevpoint = poly.points[:-1]
        #nr6 = prevpoint[6]
        #rospy.loginfo(prevpoint[6])
        

        for i in range(len(poly.points)):
            rospy.loginfo("switch line")
            x_ind = int((poly.points[i].x - self.grid.info.origin.position.x) / self.grid.info.resolution)
            y_ind = int((poly.points[i].y - self.grid.info.origin.position.y) / self.grid.info.resolution)
            self.matrix[y_ind, x_ind] = 6
            if i == 0:
                #rospy.loginfo(poly.points[0])
                continue
            
            
            else:
                #rospy.loginfo(prevpoint[i-1])
                #rospy.loginfo(poly.points[i])
                
                if poly.points[i].x < prevpoint[i-1].x and poly.points[i].y > prevpoint[i-1].y:#negative k
                    rospy.loginfo(prevpoint[i-1])
                    tx = prevpoint[i-1].x
                    ty = prevpoint[i-1].y
                    bx = poly.points[i].x
                    by = poly.points[i].y
                    angle = math.atan2(ty-by, tx-bx)
                    minitargetx = bx + margin*math.cos(angle)
                    minitargety = by + margin*math.sin(angle)
                    rospy.loginfo("1")
                    while (tx-bx) * (tx-minitargetx) > 0 and (ty-by) * (ty-minitargety) > 0:
                        x_ind = int((minitargetx - self.grid.info.origin.position.x) / self.grid.info.resolution)
                        y_ind = int((minitargety - self.grid.info.origin.position.y) / self.grid.info.resolution)
                        self.matrix[y_ind, x_ind] = 6
                        bx = minitargetx
                        by = minitargety
                        minitargetx = bx + margin*math.cos(angle)
                        minitargety = by + margin*math.sin(angle)
                    rospy.loginfo(prevpoint[i-1])
                elif poly.points[i].x < prevpoint[i-1].x and poly.points[i].y < prevpoint[i-1].y: #positive k
                    rospy.loginfo(prevpoint[i-1])
                    tx = prevpoint[i-1].x
                    ty = prevpoint[i-1].y
                    bx = poly.points[i].x
                    by = poly.points[i].y
                    angle = math.atan2(ty-by, tx-bx)
                    minitargetx = bx + margin*math.cos(angle)
                    minitargety = by + margin*math.sin(angle)
                    rospy.loginfo("2")
                    while (tx-bx) * (tx-minitargetx) > 0 and (ty-by) * (ty-minitargety) > 0:
                        x_ind = int((minitargetx - self.grid.info.origin.position.x) / self.grid.info.resolution)
                        y_ind = int((minitargety - self.grid.info.origin.position.y) / self.grid.info.resolution)
                        self.matrix[y_ind, x_ind] = 6
                        bx = minitargetx
                        by = minitargety
                        minitargetx = bx + margin*math.cos(angle)
                        minitargety = by + margin*math.sin(angle)
                    rospy.loginfo(prevpoint[i-1])
                elif poly.points[i].x > prevpoint[i-1].x and poly.points[i].y > prevpoint[i-1].y: #positive k
                    rospy.loginfo(prevpoint[i-1])
                    tx = poly.points[i].x
                    ty = poly.points[i].y
                    bx = prevpoint[i-1].x
                    by = prevpoint[i-1].y
                    angle = math.atan2(ty-by, tx-bx)
                    minitargetx = bx + margin*math.cos(angle)
                    minitargety = by + margin*math.sin(angle)
                    rospy.loginfo("3")
                    while (tx-bx) * (tx-minitargetx) > 0 and (ty-by) * (ty-minitargety) > 0:
                        x_ind = int((minitargetx - self.grid.info.origin.position.x) / self.grid.info.resolution)
                        y_ind = int((minitargety - self.grid.info.origin.position.y) / self.grid.info.resolution)
                        self.matrix[y_ind, x_ind] = 6
                        bx = minitargetx
                        by = minitargety
                        minitargetx = bx + margin*math.cos(angle)
                        minitargety = by + margin*math.sin(angle)
                    rospy.loginfo(prevpoint[i-1])
                elif poly.points[i].x > prevpoint[i-1].x and poly.points[i].y < prevpoint[i-1].y:# negative k
                    rospy.loginfo(prevpoint[i-1])
                    tx = poly.points[i].x
                    ty = poly.points[i].y
                    bx = prevpoint[i-1].x
                    by = prevpoint[i-1].y
                    rospy.loginfo(prevpoint[i-1])
                    rospy.loginfo(poly.points[i])
                    angle = math.atan2(ty-by, tx-bx)
                    minitargetx = bx + margin*math.cos(angle)
                    minitargety = by + margin*math.sin(angle)
                    rospy.loginfo("4")
                    while (tx-bx) * (tx-minitargetx) > 0 and (ty-by) * (ty-minitargety) > 0:
                        x_ind = int((minitargetx - self.grid.info.origin.position.x) / self.grid.info.resolution)
                        y_ind = int((minitargety - self.grid.info.origin.position.y) / self.grid.info.resolution)
                        self.matrix[y_ind, x_ind] = 6
                        bx = minitargetx
                        by = minitargety
                        minitargetx = bx + margin*math.cos(angle)
                        minitargety = by + margin*math.sin(angle)
                    rospy.loginfo(prevpoint[i-1])
                elif poly.points[i].x == prevpoint[i-1].x:
                    if poly.points[i].y > prevpoint[i-1].y:
                        tx = poly.points[i].x
                        ty = poly.points[i].y
                        bx = prevpoint[i-1].x
                        by = prevpoint[i-1].y
                    else: 
                        tx = prevpoint[i-1].x
                        ty = prevpoint[i-1].y
                        bx = poly.points[i].x
                        by = poly.points[i].y
                    minitargetx = bx
                    minitargety = by + margin
                    while (ty-by) * (ty-minitargety) > 0:
                        x_ind = int((minitargetx - self.grid.info.origin.position.x) / self.grid.info.resolution)
                        y_ind = int((minitargety - self.grid.info.origin.position.y) / self.grid.info.resolution)
                        self.matrix[y_ind, x_ind] = 6
                        bx = minitargetx
                        by = minitargety
                        minitargetx = bx
                        minitargety = by + margin
                elif poly.points[i].y == prevpoint[i-1].y:
                    if poly.points[i].x > prevpoint[i-1].x:
                        tx = poly.points[i].x
                        ty = poly.points[i].y
                        bx = prevpoint[i-1].x
                        by = prevpoint[i-1].y
                    else: 
                        tx = prevpoint[i-1].x
                        ty = prevpoint[i-1].y
                        bx = poly.points[i].x
                        by = poly.points[i].y
                    minitargetx = bx + margin
                    minitargety = by
                    while (tx-bx) * (tx-minitargetx) > 0:
                        x_ind = int((minitargetx - self.grid.info.origin.position.x) / self.grid.info.resolution)
                        y_ind = int((minitargety - self.grid.info.origin.position.y) / self.grid.info.resolution)
                        self.matrix[y_ind, x_ind] = 6
                        bx = minitargetx
                        by = minitargety
                        minitargetx = bx + margin
                        minitargety = by
    
    def __getImage(self) -> np.array:
        image = np.array([[(255, 255, 255) for i in range(self.grid.info.width)] for i in range(self.grid.info.height)], dtype=np.uint8)
        for i in range(self.grid.info.width):
            for ii in range(self.grid.info.height):
                if self.grid.data[i,ii] == 0:
                    image[i,ii] = (255, 255, 255)
                elif self.grid.data[i,ii] == 1:
                    image[i,ii] = (0, 0, 0)
                elif self.grid.data[i,ii] == 2:
                    image[i,ii] = (0, 0, 255)
                elif self.grid.data[i,ii] == 3:
                    image[i,ii] = (0, 255, 0)
        return image
    def doAnimate(self):
        
        plt.imshow(self.__getImage())
        plt.show()
    

  

    def __doObjectCallback(self, msg: objectPoseStampedLst):
        listOfPoses: List[PoseStamped] = msg.PoseStamped
        # print(msg.object_class)
        labels: List[str] = [self.class_dictionary[c] for c in msg.object_class]
        for i in range(len(listOfPoses)):
            c = labels[i]
            pose = listOfPoses[i]
            x = pose.pose.position.x
            y = pose.pose.position.y
            x_ind = int((x - self.grid.info.origin.position.x) / self.grid.info.resolution)
            y_ind = int((y - self.grid.info.origin.position.y) / self.grid.info.resolution)
            if x_ind>self.grid.info.height or y_ind>self.grid.info.width:
                    print("FUCKUP!NOTGOOD!VERY BAD!!!")
                    exit()
            try:
                if self.matrix[y_ind, x_ind] != 5:
                    self.matrix[y_ind, x_ind] = c
            except IndexError:
                print("Outside grid")
                exit()
            

    def __doWorkspaceCallback(self, msg: Marker):
        poly = msg.points[:-1]
        poly_real = []
        for i in range(len(poly)):
            poly_real.append((poly[i].x,poly[i].y))

        for i in range(self.grid.info.width):
            for j in range(self.grid.info.height):
                transform = self.tf_buffer.lookup_transform("arucomap", "map", rospy.Time(0), rospy.Duration(2))
                point = PoseStamped()
                point.pose.position.x = self.grid.info.origin.position.x + self.grid.info.resolution*i
                point.pose.position.y = self.grid.info.origin.position.y + self.grid.info.resolution*j
                point.pose.orientation= Quaternion(0,0,0,1)
                point.header.frame_id="map"
                #point =  tf2_geometry_msgs.do_transform_pose(point, transform)
                #inside = self.point_inside_polygon(point.pose.position.x, point.pose.position.y, poly)
                path = mpltPath.Path(poly_real)
                inside = path.contains_points([[point.pose.position.x,point.pose.position.y]])
                if not inside and self.matrix[j,i]!=6:
                    self.matrix[j,i]=5
            
        self.anchordetected = True
        start = Int64()
        start.data = 1
        self.start_explore.publish(start)