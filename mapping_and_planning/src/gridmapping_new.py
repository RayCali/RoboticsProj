#!/usr/bin/python3
import rospy
import numpy as np
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt

class Map:
    def __init__(self.grid, plot=False, width=100, height=100, resolution=0.05):
        self.grid.map = OccupancyGrid()
        self.grid.map.header.frame_id = "map"
        self.grid.map.info.resolution = resolution
        self.grid.map.info.width = width
        self.grid.map.info.height = height
        self.grid.naming_convention = {
            "unknown": 0,
            "free": 1,
            "occupied": 2,
            "toy": 3,
            "box": 4
        }
        self.grid.map.info.origin = Pose(Point(-2.5, -2.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.grid.map.data = np.zeros((self.grid.map.info.width, self.grid.map.info.height), dtype=np.uint8)
        self.grid.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.grid.scan_sub = rospy.Subscriber("/scan", LaserScan, self.grid.__doScanCallback)#Should be point cloud PointCloud2
        self.grid.imageSub = rospy.Subscriber("/detection/pose", objectPoseStamped, doUpdate)

        if plot:
            self.grid.__doDrawBox()

    def __doScanCallback(self, msg: LaserScan):
            rate = rospy.Rate(10.0)
            try:
                transform = tf_buffer.lookup_transform("map", "camera_link", latestupdate, rospy.Duration(2))
                new_aruco_pose = tf2_geometry_msgs.do_transform_pose(arucopose, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo(e)
        #look up transform from laser to map
        # TODO: ask Rayan what happens here below
        #addera x och y med robotens position och robotens yaw
            for i in range(len(msg.ranges)):
                if msg.ranges[i] < 3.0:
                    x = msg.ranges[i] * np.cos(msg.angle_min + i * msg.angle_increment)
                    y = msg.ranges[i] * np.sin(msg.angle_min + i * msg.angle_increment)
                    x_ind = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
                    y_ind = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
                    self.map.data[y_ind * self.map.info.width + x_ind] = 100
                

            self.map.header.stamp = rospy.Time.now()
        # self.map_pub.publish(self.map)
    
    def __doDrawBox(self):
        boxSize = 100
        lower = 100
        upper = 200
        for i in range(lower, lower + boxSize, 1):
            for ii in range(lower, lower + boxSize, 1):
                self.map.data[i, ii] = 1
    
    def __getImage(self) -> np.array:
        image = np.array([[(255, 255, 255) for i in range(self.map.info.width)] for i in range(self.map.info.height)], dtype=np.uint8)
        for i in range(self.map.info.width):
            for ii in range(self.map.info.height):
                if self.map.data[i,ii] == 0:
                    image[i,ii] = (255, 255, 255)
                elif self.map.data[i,ii] == 1:
                    image[i,ii] = (0, 0, 0)
                elif self.map.data[i,ii] == 2:
                    image[i,ii] = (0, 0, 255)
                elif self.map.data[i,ii] == 3:
                    image[i,ii] = (0, 255, 0)
        return image
    def doAnimate(self):
        
        plt.imshow(self.__getImage())
        plt.show()
    

  
