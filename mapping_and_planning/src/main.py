#!/usr/bin/python3

import numpy as np
import math
import rospy
from rospy import loginfo
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
# import torch
# from torchvision import transforms
from geometry_msgs.msg import PoseStamped, Pose
from msg_srv_pkg.msg import objectPoseStampedLst
from msg_srv_pkg.srv import Moveto, MovetoResponse
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
        self.pathPlanner_srv = rospy.Service("pathPlanner", Moveto, self.doPathPlanner)
    
        self.path_pub = rospy.Publisher("/planning/path", Path, queue_size=10)

    def doPathPlanner(self, req: Moveto) -> MovetoResponse:
        try:
            # should be from base link to grid
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
        x = transform.transform.translation.x - self.map.grid.info.origin.position.x
        y = transform.transform.translation.y - self.map.grid.info.origin.position.y
        start = (x,y)
        goal = (req.goal.pose.position.x - self.map.grid.info.origin.position.x, req.goal.pose.position.y - self.map.grid.info.origin.position.y)
        self.rrt = RRTStar(
            start=start,
            goal=goal,
            obstacles=self.getObstacles(),
            inside = self.getInside(),
            width=self.map.grid.info.width,
            height=self.map.grid.info.height,
            )
        self.rrt.doPath()
        if self.rrt.getPathFound:

            path_msg = Path()
            path: List[List[float, float]] = self.rrt.getPath()
            for point in path:
                path_msg.poses.append(self.getPoseStamped(point))
            self.path_pub.publish(path_msg)
        return MovetoResponse(SUCCESS)
    
    def getInside(self):
        matrix_with_true_or_false_statements_depending_on_if_the_cell_is_inside_of_the_workspace = self.map.matrix != 5
        return matrix_with_true_or_false_statements_depending_on_if_the_cell_is_inside_of_the_workspace 

    def getObstacles(self):
        obstacles = zip(np.where(self.map.matrix == 2))
        offset = self.map.grid.info.resolution
        return [(x+offset,y+offset) for x,y in obstacles]
    def getPoseStamped(self, point: List[float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0] - self.map.grid.info.origin.position.x
        pose.pose.position.y = point[1] - self.map.grid.info.origin.position.y
        return pose
        

if __name__ == "__main__":

    rospy.init_node("mapping_and_planning")
        
    m = Map(True, 11, 11, 0.05)
    rospy.sleep(1)
    m.doPublish()
    # print(getMostValuedCell(m.matrix, m.grid.info.width, m.grid.info.height))
    while not rospy.is_shutdown():
        rospy.sleep(0.05)
        m.doPublish()
        
