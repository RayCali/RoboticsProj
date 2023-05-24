#!/usr/bin/python3
import numpy as np
from typing import List
from random import random
from nav_msgs.msg import OccupancyGrid
from time import time
# https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
# Glömde att lägga till noden i self.nodes listan
class Node:
    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.parent: Node = None
        self.cost = 0.0
    def getDistTo(self, node):
        return np.sqrt(
            np.square(self.x - node.x) + np.square(self.y - node.y)
        )
    def setParent(self, node):
        self.parent = node
        self.cost = self.getDistTo(self.parent) + self.parent.cost
        

class RRTStar:
    def __init__(
            self, 
            start: List[float],
            goal: List[float],
            obstacles: List[List[float]],
            inside: List[List[float]],
            width: float,
            height: float,
            grid: OccupancyGrid,
            r: float, #radius to osbtacle
            pick: float = 0.2, #
            proximity: float = 1.0 #The proximity at which we will look for a new parent
            ) -> None:
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = [Node(pos) for pos in obstacles]
        self.inside = inside
        self.width = width
        self.height = height
        self.grid = grid
        self.nodes = [self.start]
        self.pick = pick
        self.r = r
        self.proximity = proximity
        self.neighbors = List[Node]
        self.goalInList = False
    
    def doPath(self, max_time = 30):
        start = time()
        i = 0
        while True:
            i += 1
            #print(i, len(self.nodes))
            node = self.getRandomNode()
            if node is None:
                continue
            
            closest = self.getClosestNode(node)
            if self.getObstacleInTheWay(node, closest):
                continue
            if node.getDistTo(self.goal) < self.r:
                #This is the goal
                if not self.goalInList:
                    self.goalInList = True
                else:
                    continue
            
            node.setParent(closest)
            self.nodes.append(node)
            self.setNeighbors(node)
            self.setParentToSomeoneWithBetterCostPlusDistance(node)
            self.doRewire(node)
            if (time() - start) > max_time:
                break
    def getPathFound(self) -> bool:
        if self.goal.parent:
            return True
        return False
    def getPath(self) -> List[List[float]]:
        path = []
        node = self.goal
        import time
        start = time.time()
        while True:
            point = [node.x, node.y]
            path.append(point)
            node = node.parent
            if (time.time() - start) > 10:
                print("loop")
                break
            if node is None:
                break
        path.reverse()
        return path
    def getPathRewired(self) -> List[List[float]]:
        node = self.goal
        while True:
            #print(np.round(node.x,2),np.round(node.y,2))
            parent = node.parent
            if parent is None:
                break
            if parent.parent is None:
                break
            while True:
                grandparent = parent.parent
                if grandparent is None:
                    break
                if self.getObstacleInTheWay(node, grandparent):
                    #print(np.round(grandparent.x, 2), np.round(grandparent.y,2),"in the way")
                    node = parent
                    break
                else:
                    #print(np.round(grandparent.x, 2), np.round(grandparent.y,2),"clear")
                    node.parent = grandparent
                    parent = grandparent
                    
        return self.getPath()


        return

    def doRewire(self, node:Node):
        for neighbor in self.neighbors:
            if node.cost + node.getDistTo(neighbor) < neighbor.cost :
                if not self.getObstacleInTheWay(node, neighbor):
                    neighbor.setParent(node)

        return    
    def setParentToSomeoneWithBetterCostPlusDistance(self, node: Node):
        min_path_cost = node.cost
        for neighbor in self.neighbors:
            path_cost_through_neighbor = node.getDistTo(neighbor) + neighbor.cost
            if path_cost_through_neighbor < min_path_cost:
                if not self.getObstacleInTheWay(node, neighbor):
                    node.setParent(neighbor)
                    min_path_cost = node.cost
        return
    def setNeighbors(self, node: Node):
        self.neighbors = []
        for neighbor in self.nodes:
            if neighbor.getDistTo(node) < self.proximity:
                #if neighbor is node.parent:
                #    continue
                self.neighbors.append(neighbor)
        return
                    
    def getObstacleInTheWay(self, node: Node, closest: Node) -> bool:
        R = int(node.getDistTo(closest) / self.r) + 2
        dx = node.x - closest.x
        dy = node.y - closest.y
        # TODO: steps are too big
        for i in range(1, R, 1):                    
            xi = np.round(closest.x + dx * i / R, 4)                
            yi = np.round(closest.y + dy * i / R, 4)
            x_ind = int(xi / self.grid.info.resolution)
            y_ind = int(yi / self.grid.info.resolution)             
            n = Node((xi,yi))                       
            for obstacle in self.obstacles:         
                # print(
                #     (closest.x, closest.y),
                #     (n.x, n.y),
                #     (node.x, node.y),
                #     (obstacle.x, obstacle.y),
                #     obstacle.getDistTo(n))
                if obstacle.getDistTo(n) < self.r or not self.inside[y_ind, x_ind]:  #
                    return True
        return False


    def getRandomNode(self):
        if random() > self.pick:
            x = random() * self.width
            y = random() * self.height
            # Obstacles are in the map frame, I think, while the new dots are in the grid frame
            x_ind = int(x / self.grid.info.resolution)
            y_ind = int(y / self.grid.info.resolution)
            node = Node(
                (x,y)
            )
            if self.getIsInObstacle(node) or not self.inside[y_ind, x_ind]:
                return None
            return node
        else:
            return self.goal

    def getIsInObstacle(self, node: Node) -> bool:
        for obstacle in self.obstacles:
            if obstacle.getDistTo(node) < self.r:
                return True
        return False
    def getClosestNode(self, node: Node) -> Node:
        closest, distance = self.start, self.start.getDistTo(node)
        for neighbour in self.nodes:
            if neighbour.getDistTo(node) < distance:
                closest, distance = neighbour, neighbour.getDistTo(node)
        return closest
    
