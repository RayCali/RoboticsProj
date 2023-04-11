#!/usr/bin/python3
from supermaps import *
from geometry_msgs.msg import PoseStamped
from msg_srv_pkg.msg import objectPoseStampedLst
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
from typing import List
import matplotlib.path as mpltPath
from config import SUCCESS, RUNNING, FAILURE
from visualization_msgs.msg import Marker

class Map(SuperMap):
    def __init__(self, plot=False, width=1000, height=1000, resolution=0.1):
        super().__init__(plot, width,height,resolution)
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
                point =  tf2_geometry_msgs.do_transform_pose(point, transform)
                #inside = self.point_inside_polygon(point.pose.position.x, point.pose.position.y, poly)
                path = mpltPath.Path(poly_real)
                inside = path.contains_points([[point.pose.position.x,point.pose.position.y]])
                if not inside:
                    self.matrix[j,i]=5
            
        self.anchordetected = True
        
        
        # workspace = msg.points[:-1]
        # index = 3
        # point = PoseStamped()
        # point.pose.position.x = workspace[index].x
        # point.pose.position.y = workspace[index].y
        # point.pose.orientation = Quaternion(0,0,0,1)
        # point.header.frame_id= "arucomap"
        # transform = self.tf_buffer.lookup_transform("map", "arucomap", rospy.Time(0), rospy.Duration(2))
        # point =  tf2_geometry_msgs.do_transform_pose(point, transform)
        # roll,pitch,yaw = tf_conversions.transformations.euler_from_quaternion([0,0,0,1])
        # yaw = yaw + math.pi/2
        # q = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
        # rospy.loginfo(q)
        # # q = Quaternion(0,0,0,1)
        # self.grid.info.origin = Pose(Point(point.pose.position.x, point.pose.position.y, 0.0), Quaternion(q[0],q[1],q[2],q[3]))
        
        # for pose in msg.poses:
        #     x = pose.position.x
        #     y = pose.position.y
        #     x_ind = int((x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        #     y_ind = int((y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        #     self.matrix[y_ind, x_ind] = 2
