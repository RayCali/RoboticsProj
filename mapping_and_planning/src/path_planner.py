#!/usr/bin/python3
import math
import random
import numpy as np
import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from gridmapping import Map

start= 0.0

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, obstacles, width, height, goal,max_iter=1000, goal_sample_rate=0.05, radius=10.0):
        self.start = Node(start[0], start[1])
        self.obstacles = obstacles
        self.width = width
        self.height = height
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.radius = radius
        self.nodes = [self.start]
        self.path = None
        
    def planning(self):
        for i in range(self.max_iter):
            # sample a new node
            if random.random() > self.goal_sample_rate:
                x = random.uniform(0, self.width)
                y = random.uniform(0, self.height)
                node = Node(x, y)
            else:
                node = self.goal
            
            # find nearest node in the tree
            nearest_node = self.nodes[0]
            for n in self.nodes:
                if self.distance(n, node) < self.distance(nearest_node, node):
                    nearest_node = n
            
            # extend the tree towards the new node

            if self.check_collision(nearest_node, node):
                new_node = self.steer(nearest_node, node)
                if new_node is not None:
                    near_nodes = self.find_near_nodes(new_node)
                    parent_node = nearest_node
                    min_cost = parent_node.cost + self.distance(parent_node, new_node)
                    for n in near_nodes:
                        if self.check_collision(n, new_node) and parent_node.cost + self.distance(parent_node, n) < min_cost:
                            min_cost = parent_node.cost + self.distance(parent_node, n)
                            parent_node = n
                    new_node.cost = min_cost
                    new_node.parent = parent_node
                    self.nodes.append(new_node)
                    self.rewire(new_node, near_nodes)
                    
                    # check if goal is reached
                    if self.distance(new_node, self.goal) < self.radius:
                        self.path = self.generate_path(new_node)
                        break
                        
        return self.path
                    
    def distance(self, n1, n2):
        dx = n1.x - n2.x
        dy = n1.y - n2.y
        print(math.sqrt(dx**2 + dy**2)) # this is the distance between the nodes. Returns a float. Remove this if you dont want to see it print every time you call it.
        return math.sqrt(dx**2 + dy**2)
    
    def check_collision(self, n1, n2):
        for o in self.obstacles:
            if self.distance_to_obstacle(n1, n2, o) < self.radius:
                return False
        return True
    
    def steer(self, n1, n2):
        if self.distance(n1, n2) > self.radius:
            new_node = Node(n1.x + self.radius * (n2.x - n1.x) / self.distance(n1, n2), n1.y + self.radius * (n2.y - n1.y) / self.distance(n1, n2))
            return new_node
        else:
            return None
        
    
    def find_near_nodes(self, new_node):
        near_nodes = []
        for n in self.nodes:
            if self.distance(n, new_node) < self.radius:
                near_nodes.append(n)
        return near_nodes
    
    def generate_path(self, new_node):
        path = []
        node = new_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]

    def rewire(self, new_node, near_nodes):
        for n in near_nodes:
            if self.check_collision(n, new_node) and new_node.cost + self.distance(new_node, n) < n.cost:
                n.parent = new_node
                n.cost = new_node.cost + self.distance(new_node, n)
                self.rewire(n, near_nodes)
        


    def distance_to_obstacle(self, n1, n2, o):
        x1 = n1.x
        y1 = n1.y
        x2 = n2.x
        y2 = n2.y
        xo = o[0]
        yo = o[1]
        r = o[2]
        dx = x2 - x1
        dy = y2 - y1
        A = dx**2 + dy**2
        B = 2 * (dx * (x1 - xo) + dy * (y1 - yo))
        C = (x1 - xo)**2 + (y1 - yo)**2 - r**2
        D = B**2 - 4 * A * C
        return (-B - math.sqrt(D)) / (2 * A) if D >= 0 else float("inf")
        #Calculates the distance from the starting position (x1, y1) to the closest point on the circumference of the obstacle, taking into account the obstacle's radius (r) and position (xo, yo).


    def doGoalCallback(self, goal:PoseStamped):
        self.goal = [goal.pose.position.x, goal.pose.position.y]
        rospy.loginfo("Goal received")
        rospy.loginfo(goal)
        self.planning()


def doPlan(m: Map):
    global start
    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
    listener = tf2_ros.TransformListener(tf_buffer)
    
    print("Starting planner... ")
    try:
        transform = tf_buffer.lookup_transform("map", "center_robot", rospy.Time(0), rospy.Duration(0.5))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)
    start_x = transform.transform.translation.x
    start_y = transform.transform.translation.y
    start = [start_x, start_y]
    x,y = np.where(m.matrix==2)
    radius = 1
    obstacles = [[o[0], o[1], radius] for o in zip(x,y)] #obstacles are represented as [x, y, radius]

    rrt = RRTStar(start=start, obstacles=obstacles, width=11, height=11)
    
    goal_sub = rospy.Subscriber("/detection/pose", PoseStamped, rrt.doGoalCallback, queue_size=1)
    print("Done!!!")
