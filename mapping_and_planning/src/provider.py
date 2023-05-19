#!/usr/bin/python3

import numpy as np
import math
import rospy
from rospy import loginfo
import tf2_ros
from msg_srv_pkg.msg import objectPoseStampedLst
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose
from msg_srv_pkg.srv import Request, RequestResponse
from gridmapping import Map
from global_explorer import getMostValuedCell
from nav_msgs.msg import Path
from planner import Node, RRTStar
from typing import List
from config import SUCCESS, RUNNING, FAILURE



class PathProvider:
    def __init__(self, map: Map) -> None:
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map: Map = map
        self.pathPlannerEx_srv = rospy.Service("/srv/doPlanpathToy/mapping_and_planning/memory", Request, self.doPlanResponse)
        # 
        self.moveto_pub = rospy.Publisher("/pathprovider/rrt", PoseStamped,  queue_size=10)
        self.moveto_sub = rospy.Subscriber("/pathprovider/rrt", PoseStamped, self.doPlanPath, queue_size=10)

        # self.goal_Ex_sub = rospy.Subscriber("/mostValuedCell", PoseStamped, self.getGoalEx, queue_size=10)
        # self.goal_toy_sub = rospy.Subscriber("/toyPoseMap", objectPoseStampedLst, self.getToy, queue_size=10)
        # self.goal_box_sub = rospy.Subscriber("/boxPoseMap", objectPoseStampedLst, self.getBox, queue_size=10)
        self.goal_sub = rospy.Subscriber("/goalTarget", objectPoseStampedLst, self.doSetGoal, queue_size=10)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=10)
        self.rewired_pub = rospy.Publisher("/rewired", Path, queue_size=10)
        self.getObstacles()
        self.STATE = RUNNING
        self.running = False
        self.goal = None
        self.planned = False
    
    def doSetGoal(self, msg: objectPoseStampedLst):
        print("got goal: ", msg)
        self.goal = msg.object_class[0]
        self.planned = False
    
    def doPlanResponse(self, req: Request):
        if self.planned:
            return RequestResponse(SUCCESS)
        if not self.running:
            if self.goal is None:
                return RequestResponse(FAILURE)
            self.running = True
            self.moveto_pub.publish(self.goal_Ex)
            return RequestResponse(RUNNING)
        if self.running:
            if self.STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.STATE == FAILURE:
                self.reset()
                return RequestResponse(FAILURE)
            if self.STATE == SUCCESS:
                self.reset()
                self.planned = True
                return RequestResponse(SUCCESS)
    def reset(self):
        self.running = False
        self.goal_Ex = None
        self.goal_toy = None
        self.goal_box = None

    
        
    def doPlanPath(self, goal: PoseStamped):
        path_msg= Path()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        path_msg.header = header
        try:
            # should be from base link to grid
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2)) #base_link or center_robot? dont know which one to use.
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
        x = transform.transform.translation.x - self.map.grid.info.origin.position.x
        y = transform.transform.translation.y - self.map.grid.info.origin.position.y
        
        start = (
            transform.transform.translation.x - self.map.grid.info.origin.position.x,
            transform.transform.translation.y - self.map.grid.info.origin.position.y
        )
        goal = (
            goal.pose.position.x - self.map.grid.info.origin.position.x, 
            goal.pose.position.y - self.map.grid.info.origin.position.y
        )
        rrt = RRTStar(
            start=start,
            goal=goal,
            obstacles=self.getObstacles(),
            inside = self.getInside(),
            width=self.map.grid.info.width * self.map.grid.info.resolution,
            height=self.map.grid.info.height * self.map.grid.info.resolution,
            grid=self.map.grid
            )
        print("Planning path")
        rrt.doPath(max_time=5)
        if rrt.getPathFound():
            print("path found")

            path: List[List[float, float]] = rrt.getPath()
            for point in path:
                path_msg.poses.append(self.getPoseStamped(point,header))
            self.path_pub.publish(path_msg)
            path_msg.poses = []
            path: List[List[float, float]] = rrt.getPathRewired()
            for point in path:
                path_msg.poses.append(self.getPoseStamped(point,header))
            self.rewired_pub.publish(path_msg)
            self.running = False
            self.STATE = SUCCESS
            return            
            

        print("no path found")
        self.running = False
        self.STATE = FAILURE
    
    def getInside(self):
        matrix_with_true_or_false_statements_depending_on_if_the_cell_is_inside_of_the_workspace = self.map.matrix != 5
        return matrix_with_true_or_false_statements_depending_on_if_the_cell_is_inside_of_the_workspace 

    def getObstacles(self):
        x, y = np.where((self.map.matrix == 2) | (self.map.matrix == 6))
        obstacles = zip(x,y)
        obstacles = [
            (np.round(o[1] * self.map.grid.info.resolution, 3),
             np.round(o[0] * self.map.grid.info.resolution, 3))
                      for o in obstacles]
        return obstacles
    def getPoseStamped(self, point: List[float], header: Header) -> PoseStamped:
        ps = PoseStamped()
        ps.header = header
        ps.pose.position.x = point[0] + self.map.grid.info.origin.position.x
        ps.pose.position.y = point[1] + self.map.grid.info.origin.position.y
        return ps
        