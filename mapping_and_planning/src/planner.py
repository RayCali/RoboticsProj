#!/usr/bin/python3
import numpy as np
from typing import List
from random import random
# https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
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
            pick: float = 0.2,
            r: float = 0.15, #radius to osbtacle
            proximity: float = 0.5 #The proximity at which we will look for a new parent
            ) -> None:
        self.start = Node(start)
        self.goal = Node(goal)
        self.obstacles = [Node(pos) for pos in obstacles]
        self.inside = inside
        self.width = width
        self.height = height
        self.nodes = [self.start]
        self.pick = pick
        self.r = r
        self.proximity = proximity
        self.neighbors = List[Node]
    
    def doPath(self, vertices = 500):
        for i in range(vertices):
            node = self.getRandomNode()
            closest = self.getClosestNode(node)
            if self.getObstacleInTheWay(node, closest):
                continue
            node.setParent(closest)
            self.setNeighbors(node)
            self.setParentToSomeoneWithBetterCostPlusDistance(node)
            self.doRewire(node)
    def getPath(self) -> List[List[float]]:
        path = []
        node = self.goal
        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        return path.reverse()

    def doRewire(self, node:Node):
        for neighbor in self.neighbors:
            if node.cost < neighbor.cost:
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
                    min_path_cost = node.cost + node.parent.cost
        return
    def setNeighbors(self, node: Node):
        self.neighbors = []
        for neighbor in self.nodes:
            if neighbor.getDistTo(node) < self.proximity:
                if neighbor is node.parent:
                    continue
                self.neighbors.append(neighbor)
        return
                    
    def getObstacleInTheWay(self, node: Node, closest: Node) -> bool:
        R = int(node.getDistTo(closest) / self.r)
        dx = node.x - closest.x
        dy = node.y - closest.y
        for i in range(1, R, 1):                    #
            xi = node.x + dx * i / R                #
            yi = node.y + dy * i / R                #
            n = Node((xi,yi))                       #
            for obstacle in self.obstacles:         #
                if obstacle.getDistTo(n) < self.r:  #
                    return True
        return False


    def getRandomNode(self):
        if random() > self.pick:
            x = random() * self.width; y = random() * self.height 
            x_ind = int((x - self.grid.info.origin.position.x) / self.grid.info.resolution)
            y_ind = int((y - self.grid.info.origin.position.y) / self.grid.info.resolution)
            node = Node(
                (x,y)
            )
            if self.getIsInObstacle(node) or not self.inside[x_ind, y_ind]:
                node =  self.getRandomNode()
        else:
            node = self.goal
        return node

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
    
