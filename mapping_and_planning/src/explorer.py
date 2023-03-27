
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from frontier_exploration.msg import Frontier, FrontierArray
from nav_msgs.msg import OccupancyGrid
import numpy as np

class FrontierExplorer:

    def __init__(self):
        # Subscribe to the map topic
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        # Subscribe to the frontier topic
        rospy.Subscriber("/frontier", FrontierArray, self.frontier_callback)

        # Initialize variables
        self.map = None
        self.frontiers = []
        self.current_goal = None

        # Create a SimpleActionClient for the move_base node
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def map_callback(self, data: OccupancyGrid):
        # Convert the OccupancyGrid message to a numpy array
        self.map = np.array(data.data).reshape((data.info.height, data.info.width))

    def frontier_callback(self, data: FrontierArray):
        # Store the frontiers
        self.frontiers = data.frontiers

    def explore(self):
        # Choose the frontier closest to the robot
        if len(self.frontiers) > 0:
            frontier_distances = []
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = "base_link"
            robot_pose.pose.position.x = 0
            robot_pose.pose.position.y = 0
            robot_pose.pose.position.z = 0
            robot_pose.pose.orientation.w = 1
            for frontier in self.frontiers:
                frontier_pose = PoseStamped()
                frontier_pose.header.frame_id = "map"
                frontier_pose.pose.position.x = frontier.centroid[0]
                frontier_pose.pose.position.y = frontier.centroid[1]
                frontier_pose.pose.position.z = 0
                frontier_distances.append(np.linalg.norm(np.array([frontier.centroid[0], frontier.centroid[1]]) - np.array([robot_pose.pose.position.x, robot_pose.pose.position.y])))
            closest_frontier = self.frontiers[np.argmin(frontier_distances)]

            # If the closest frontier is not the current goal, set the goal
            if self.current_goal is None or closest_frontier.centroid != self.current_goal.target_pose.pose.position:
                self.current_goal = MoveBaseGoal()
                self.current_goal.target_pose.header.frame_id = "map"
                self.current_goal.target_pose.pose.position.x = closest_frontier.centroid[0]
                self.current_goal.target_pose.pose.position.y = closest_frontier.centroid[1]
                self.current_goal.target_pose.pose.orientation.w = 1
                self.client.send_goal(self.current_goal)
                self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('frontier_explorer')
    fe = FrontierExplorer()
    while not rospy.is_shutdown():
        fe.explore()

# sudo apt-get update
# sudo apt-get install ros-<distro>-gmapping
# sudo apt-get install ros-<distro>-frontier-exploration