#!/usr/bin/python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2, PointField

class Map(object):
    def __init__(self):
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.resolution = 0.05
        self.map.info.width = 100
        self.map.info.height = 100
        self.map.info.origin = Pose(Point(-2.5, -2.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.map.data = [0] * self.map.info.width * self.map.info.height
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", PointCloud2, self.scan_callback)#Should be point cloud PointCloud2

    def scan_callback(self, msg: PointCloud2):
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < 3.0:
                x = msg.ranges[i] * np.cos(msg.angle_min + i * msg.angle_increment)
                y = msg.ranges[i] * np.sin(msg.angle_min + i * msg.angle_increment)
                x_ind = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
                y_ind = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
                self.map.data[y_ind * self.map.info.width + x_ind] = 100
        self.map.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map)
    
    def main(self):
        rospy.spin()

    
if __name__ == "__main__":
    print("Starting map node")
    rospy.init_node("map")
    map = Map()
    map.main()
  
