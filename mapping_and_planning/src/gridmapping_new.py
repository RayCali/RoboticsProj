#!/usr/bin/python3
import rospy
import numpy as np
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped
import matplotlib.pyplot as plt
import tf_conversions

tf_buffer = None #tf buffer length
listener = None
br = None
st = None

class Map:
    def __init__(self, plot=False, width=1000, height=1000, resolution=0.1):
        width = int(width/resolution)
        height = int(height/resolution)
        global tf_buffer, listener, br, st
        tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        listener = tf2_ros.TransformListener(tf_buffer)
        br = tf2_ros.TransformBroadcaster()
        st = tf2_ros.StaticTransformBroadcaster()
        
        self.matrix = np.zeros((width, height), dtype=np.int8)
        
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
            "box": 4
        }
        self.grid.info.origin = Pose(Point(-5.0, -5.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)) #This is the center/origin of the grid 
        self.grid.data = None
        self.grid_pub = rospy.Publisher("/topic", OccupancyGrid, queue_size=1000, latch=True)
        self.grid_sub = rospy.Subscriber("/scan", LaserScan, self.__doScanCallback)
        # self.grid_sub_detect = rospy.Subscriber("/detection/pose", PoseStamped, self.doDetectCallback)
    
        
        if plot:
            self.__doDrawBox()
    def __getOccupancyGridObject(self) -> OccupancyGrid:
        self.grid.data = []
        for i in range(self.grid.info.width):
            for ii in range(self.grid.info.height):
                self.grid.data.append(self.__getProbabilityFromMatrixValue(self.matrix[i,ii]))
        return self.grid
    
    def doPublish(self):
        rate = rospy.Rate(10.0)
        og = self.__getOccupancyGridObject()
        # rospy.loginfo(og)
        self.grid_pub.publish(og)
        rate.sleep()

    def __getProbabilityFromMatrixValue(self, x: int) -> int:
        if x == 0: # unknown 
            return 20
        if x == 1: # a bit grey
            return 0
        if x == 2:
            return 100 # black(obstacles)
        if x == 3:
            return 50
        if x == 4:
            return 90
        else:
            raise Exception("None value in matrix")


    def __doScanCallback(self, msg: LaserScan):
            global tf_buffer
            latestupdate = rospy.Time(0)
            try:
                transform = tf_buffer.lookup_transform("map", "base_link", latestupdate, rospy.Duration(2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo(e)

            anglelist = tf_conversions.transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        #look up transform from laser to map
        # TODO: ask Rayan what happens here below
        #addera x och y med robotens position och robotens yaw
            for i in range(len(msg.ranges)):
                if msg.ranges[i] < 3.0:
                    
                    x = transform.transform.translation.x + msg.ranges[i] * np.cos(msg.angle_min + i * msg.angle_increment + anglelist[2])
                    y = transform.transform.translation.y + msg.ranges[i] * np.sin(msg.angle_min + i * msg.angle_increment + anglelist[2])
                    
                    x_ind = int((x - self.grid.info.origin.position.x) / self.grid.info.resolution)
                    y_ind = int((y - self.grid.info.origin.position.y) / self.grid.info.resolution)
                    self.matrix[y_ind, x_ind] = 2
                    self.__doDrawFreespace(
                        r=msg.ranges[i], 
                        x0 = transform.transform.translation.x,
                        y0 = transform.transform.translation.y,
                        x1 = x,
                        y1 = y,
                        x_1_ind=x_ind,
                        y_1_ind=y_ind 
                        )
                    print(x,y)
                    print(x_ind, y_ind)
                    print(self.matrix.shape)
                    print(self.matrix[y_ind, x_ind])
                    print()

                

            self.grid.header.stamp = rospy.Time.now()
    def __doDrawFreespace(self, r:float, x0: float, y0: float, x1: float, y1:float, x_1_ind:int, y_1_ind:int):
        N = 100
        indices = []
        for i in range(N):
            xi = x0 + i * (x1 - x0)
            yi = y0 + i * (y1 - y0)
            x_i_ind = int((xi - self.grid.info.origin.position.x) / self.grid.info.resolution)
            y_i_ind = int((xi - self.grid.info.origin.position.y) / self.grid.info.resolution)
            if x_i_ind != x_1_ind and y_i_ind != y_1_ind:
                indices.append((x_i_ind, y_i_ind))
        
        for x,y in indices:
            self.__doCheckForFreeSpaceAndInsert(self, x, y)
        
    
    def __doCheckForFreeSpaceAndInsert(self, x:int, y:int):
        cell = self.matrix[y,x]
        if cell == 0 or cell == 1 or cell == 2: #painting over, UNK,OBS and FRE to FRE
            self.matrix[y,x] == 1
        elif cell == 3 or cell == 4: # TOY or BOX found between us and the obstacle
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
    

  
