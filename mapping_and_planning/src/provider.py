#!/usr/bin/python3

import numpy as np
import math
import rospy
from rospy import loginfo
import tf2_ros

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose
from msg_srv_pkg.srv import Moveto, MovetoResponse, Request, RequestResponse
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
        self.pathPlanner_srv = rospy.Service("pathPlanner", Request, self.doReturnMovetoResponse)

        self.moveto_pub = rospy.Publisher("/pathprovider/rrt", PoseStamped,  queue_size=10)
        self.moveto_sub = rospy.Subscriber("/pathprovider/rrt", PoseStamped, self.doPlanPath, queue_size=10)

        self.goal_sub = rospy.Subscriber("/mostValuedCell", PoseStamped, self.getGoal, queue_size=10)

        self.path_pub = rospy.Publisher("/path", Path, queue_size=10)
        self.rewired_pub = rospy.Publisher("/rewired", Path, queue_size=10)
        self.getObstacles()
        self.running = False
        self.STATE = RUNNING
    
    def getGoal(self, msg: PoseStamped):
        print("got goal: ", msg)
        self.goal = msg
    
    def doReturnMovetoResponse(self, req: Request):
        if not self.running:
            self.running = True
            self.moveto_pub.publish(self.goal)
            return RequestResponse(SUCCESS)
        if self.running:
            if self.STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)
            if self.STATE == SUCCESS:
                self.running = False
                return RequestResponse(SUCCESS)
        
    def doPlanPath(self, goal: PoseStamped):
        path_msg= Path()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        path_msg.header = header
        try:
            # should be from base link to grid
            transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(2))
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
        rrt.doPath(max_time=10)
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
            self.STATE == SUCCESS
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
        return 