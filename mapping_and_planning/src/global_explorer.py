#!/usr/bin/python3
import numpy as np
import rospy
import tf2_ros
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion, Vector3

# https://cs231n.github.io/convolutional-networks/
F = 10  # receptive field
S = 10   # stride
P = 0   # padding

# Leaving the mask here to illustrate how it looks, we can choose other values
mask: np.array = np.array([ [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

mask: np.array = np.array([[0 for i in range(F)] for i in range(F)])



def f(mask_value: int, matrix_value: int) -> int:
    if mask_value == matrix_value:
        return 1 #Unknown space
    else:
        return 0 #Obstacles

def getMostValuedCell(matrix: np.array, width: int, height: int, resolution: float, grid_to_map: Tuple[float, float]) -> List[int]:
    heuiristic_values = getHeuristicMap(matrix, mask, width, height)
    # rospy.loginfo(heuiristic_values)
    most_valued_cell = [0, 0, 0]
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    tfbroadcaster = tf2_ros.TransformBroadcaster()

    for cell in heuiristic_values:
        trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2.0))
        x_grumpy = trans.transform.translation.x 
        y_grumpy = trans.transform.translation.y
        x_cell_in_grid_frame = cell[1] * resolution
        y_cell_in_grid_frame = cell[2] * resolution
        x_cell_in_map_frame = x_cell_in_grid_frame + grid_to_map[0]
        y_cell_in_map_frame = y_cell_in_grid_frame + grid_to_map[1]
        distance_to_cell_from_grumpy = np.sqrt((x_grumpy - x_cell_in_map_frame)**2 + (y_grumpy - y_cell_in_map_frame)**2)
        cell[0] = cell[0] + 10 - distance_to_cell_from_grumpy        
        #print(cell[1:], (x_cell_in_grid_frame, y_cell_in_grid_frame), (x_cell_in_map_frame, y_cell_in_map_frame), np.round(distance_to_cell_from_grumpy,1))

        if cell[0] > most_valued_cell[0]:
            most_valued_cell = cell
        if cell[0]!=1000:
            rospy.loginfo(cell)
        
        """
        if 1e-1< distance_to_cell_from_grumpy-1.0 < 5e-1:
            
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "cell#" + str(cell[1]) + "," + str(cell[2])
            
            cell_vec3: Vector3 = Vector3()
            cell_vec3.x = x_cell_in_map_frame
            cell_vec3.y = y_cell_in_map_frame
            cell_quat = Quaternion()
            cell_quat.z = 1
            t.transform.translation = cell_vec3
            t.transform.rotation = cell_quat
            tfbroadcaster.sendTransform(t)
        """
    x_cell_in_grid_frame = most_valued_cell[1] * resolution
    y_cell_in_grid_frame = most_valued_cell[2] * resolution
    x_cell_in_map_frame = x_cell_in_grid_frame + grid_to_map[0]
    y_cell_in_map_frame = y_cell_in_grid_frame + grid_to_map[1]
    #print("Most valued cell: ", most_valued_cell, (x_cell_in_map_frame, y_cell_in_map_frame))

        
    pose_of_most_valued_cell = TransformStamped()
    pose_of_most_valued_cell.header.stamp = rospy.Time.now()
    pose_of_most_valued_cell.header.frame_id = "map"
    pose_of_most_valued_cell.child_frame_id = "exploration_goal"
    
    cell_vec3: Vector3 = Vector3()
    cell_vec3.x = x_cell_in_map_frame
    cell_vec3.y = y_cell_in_map_frame

    pose_of_most_valued_cell.transform.translation = cell_vec3
    
    cell_quat = Quaternion()
    cell_quat.z = 1
    pose_of_most_valued_cell.transform.rotation = cell_quat
    
    tfbroadcaster.sendTransform(pose_of_most_valued_cell)

    return pose_of_most_valued_cell


    
def heuristic(matrix: np.array, mask: np.array, width: int, height: int) -> int:
    area_value = 0
    for h in range(F):
        for w in range(F):
            #area_value += f(mask[h,w], matrix[h,w])
            area_value += f(mask[w,h], matrix[w,h])
    return area_value


def getHeuristicMap(matrix: np.array, mask: np.array, W: int, H: int) -> np.array:
    heuiristic_values = []
    for h in range(0, H - F, S):
        for w in range(0, W - F, S):
            #rospy.loginfo(matrix[h:h+F, w:w+F])
            #m = matrix[h:h+F, w:w+F]
            m = matrix[w:w+F, h:h+F]
            
            cell_value = heuristic(m, mask, W, H)
            heuiristic_values.append([cell_value, h, w])            
    return heuiristic_values
