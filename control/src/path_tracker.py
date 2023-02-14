#!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
# from robp_msgs.msg import Encoders
# from aruco_msgs.msg import MarkerArray
# from robp_msgs.msg import DutyCycles
# from nav_msgs.msg import Odometry
# from tf2_msgs.msg import TFMessage
# import tf_conversions
# import tf2_ros
# import tf2_geometry_msgs
# import math
# import numpy as np
# import tf


# x = 0.0
# y = 0.0
# theta = 0.0

# class path(object):
#     def __init__(self):
#         self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.tracker)
#         self.robot = rospy.Subscriber("/tf", TFMessage, self.robot_pos)
        
#         print('tracker hej')

#     def robot_pos(self, msg:TFMessage):
#         self.robot_pose = msg.transforms[0].transform.translation
#         # print(self.robot_pose.x, self.robot_pose.y)
#         return self.robot_pose
    

#     def tracker(self,msg:PoseStamped):
#         global x, y, theta
#         rate = rospy.Rate(20)
        

#         self.cam_pose = PoseStamped()
#         twist = Twist()
        

#         while not rospy.is_shutdown():
#             print('tracker hej')
#             msgTime = msg.header.stamp
#             print("Hej!")
#             self.cam_pose.header = msg.header
#             self.cam_pose.pose = msg.pose

#             broadcaster = tf2_ros.TransformBroadcaster()
#             t = TransformStamped()
  

#             try:
#                 trans_odom = tfBuffer.lookup_transform("map", "odom", msgTime, timeout=rospy.Duration(6.0))
#                 # trans_base = tfBuffer.lookup_transform("map", "base_link", msgTime, timeout=rospy.Duration(6.0))
#                 # print(trans_base)
#                 self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, trans_odom)
#                 # self.robot_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, trans_base)
#                 # print(self.goal_pose)
#                 # print(self.robot_pose)
#                 # print('jkdlsakldfjkdfalkfa')
#                 # return self.goal_pose
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 rospy.logerr("Failed to transform point from odom frame to base_link frame")
#                 pass
            
#             t.header.frame_id = "map"
#             t.child_frame_id = "goal"
#             t.header.stamp = msg.header.stamp
#             t.transform.translation = self.goal_pose.pose.position
#             t.transform.rotation = self.goal_pose.pose.orientation
           

#             # print('before x and y')

#             # print(self.goal_pose)
#             #Update the position
#             distance_x = t.transform.translation.x - self.robot_pose.x
#             distance_y = t.transform.translation.y - self.robot_pose.y
#             # inc_x = self.goal_pose.pose.position.x - x
#             # inc_y = self.goal_pose.pose.position.y - y
#             inc_x = distance_x
#             inc_y = distance_y
#             print(inc_x, inc_y)

#             if math.sqrt(inc_x**2 + inc_y**2)<0.1:
#                 print('You have reached the goal')
#                 twist.linear.x = 0.0
#                 twist.angular.z = 0.0
#                 pub_twist.publish(twist)
#                 break

#             else:
#                 angle_to_goal = math.atan2(inc_y, inc_x)
#                 twist.linear.x = 5.0
#                 twist.angular.z = angle_to_goal
#                 pub_twist.publish(twist)
#                 print('Moving towards the goal')
        
#             broadcaster.sendTransform(t)


#         rate.sleep()

# if __name__ == "__main__":
#     rospy.init_node("path_tracker")

#     tfBuffer = tf2_ros.Buffer()
#     tflistener = tf2_ros.TransformListener(tfBuffer)

#     pub_duty = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
#     pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     try:
#         planner = path()
        
#     except rospy.ROSInterruptException:
#         pass
    
#     rospy.spin()


import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
from robp_msgs.msg import Encoders
from aruco_msgs.msg import MarkerArray
from robp_msgs.msg import DutyCycles
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import tf



class path(object):
    def __init__(self):
        self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.tracker)
        # self.robot = rospy.Subscriber("/tf", TFMessage, self.robot_pos)
        
        print('tracker hej')
    def tracker(self,msg:PoseStamped):
        self.cam_pose = PoseStamped()
        twist = Twist()
        # self.robot_pose = TFMessage()

        # print("tracker function working")
        rate = rospy.Rate(20)
        msgTime = msg.header.stamp
        # print("Hej!")
        self.cam_pose.header.stamp = msg.header.stamp
        self.cam_pose.header.frame_id = msg.header.frame_id
        self.cam_pose.pose = msg.pose
        
        # print(self.robot_pose.x, self.robot_pose.y)
        # return self.robot_pose

        try:
            trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(1.0))
            self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, trans)
            # print(self.goal_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform point from odom frame to base_link frame")
            pass
        

        # t = TransformStamped()

        # self.robot_pose = msg.transforms[0].transform.translation
        # self.robot_pose = msg.transforms[0].transform.rotation  do not use
        # print(self.robot_pose)
        
        # broadcaster = tf2_ros.TransformBroadcaster()
        # t.header.frame_id = "map"
        # t.child_frame_id = "goal"
        # t.header.stamp = msgTime
        # t.transform.translation = self.goal_pose.pose.position
        # t.transform.rotation = self.goal_pose.pose.orientation
        # print('tracker hej')

        # print('before x and y')

        # print(self.goal_pose)

        distance_x = self.goal_pose.pose.position.x
        distance_y = self.goal_pose.pose.position.y
        inc_x = distance_x
        inc_y = distance_y
        # print(inc_x, inc_y)
        if math.sqrt(inc_x**2 + inc_y**2)<0.1:
            print('You have reached the goal')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub_twist.publish(twist)

        else:
            angle_to_goal = math.atan2(inc_y, inc_x)
            twist.linear.x = 5.0
            twist.angular.z = angle_to_goal
            print('Moving towards the goal')
            pub_twist.publish(twist)
        
            # broadcaster.sendTransform(t)


            rate.sleep()
            # rate.sleep()
if __name__ == "__main__":
    rospy.init_node("path_tracker")

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)

    pub_duty = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    try:
        planner = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()






# import rospy
# from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
# from robp_msgs.msg import Encoders
# from aruco_msgs.msg import MarkerArray
# from robp_msgs.msg import DutyCycles
# from nav_msgs.msg import Odometry
# from tf2_msgs.msg import TFMessage
# import tf_conversions
# import tf2_ros
# import tf2_geometry_msgs
# import math
# import numpy as np
# import tf


# x = 0.0
# y = 0.0
# theta = 0.0

# class path(object):
#     def __init__(self):
#         self.robot = rospy.Subscriber("/tf", TFMessage, self.robot_pos)
#         self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.tracker)


#     def robot_pos(self, msg:TFMessage):
#         msgTime = msg.transforms[0].header.stamp
#         try:
#             trans_odom = tfBuffer.lookup_transform("map", "odom", msgTime, timeout=rospy.Duration(6.0))
#             self.robot = tf2_geometry_msgs.do_transform_pose(self.cam_pose, trans_odom)
#             print(self.robot)
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             rospy.logerr("Failed to transform point from odom frame to base_link frame")
#             pass
#         # print(self.robot_pose.x, self.robot_pose.y)
    

#     def tracker(self,msg:PoseStamped):
#         rate = rospy.Rate(20)
        
#         self.cam_pose = PoseStamped()
#         twist = Twist()
        
#         print('tracker hej')
#         msgTime = msg.header.stamp
#         print("Hej!")
#         self.cam_pose.header = msg.header
#         self.cam_pose.pose = msg.pose

#         try:
#             trans_odom = tfBuffer.lookup_transform("map", "odom", msgTime, timeout=rospy.Duration(6.0))
#             self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, trans_odom)

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             rospy.logerr("Failed to transform point from odom frame to base_link frame")
#             pass
        

#         #Update the position
#         distance_x = self.- self.robot_pose.x
#         distance_y = t.transform.translation.y - self.robot_pose.y
#         # inc_x = self.goal_pose.pose.position.x - x
#         # inc_y = self.goal_pose.pose.position.y - y
#         inc_x = distance_x
#         inc_y = distance_y
#         print(inc_x, inc_y)

#         if math.sqrt(inc_x**2 + inc_y**2)<0.1:
#             print('You have reached the goal')
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             pub_twist.publish(twist)

#         else:
#             angle_to_goal = math.atan2(inc_y, inc_x)
#             twist.linear.x = 5.0
#             twist.angular.z = angle_to_goal
#             pub_twist.publish(twist)
#             print('Moving towards the goal')
        


#         rate.sleep()

# if __name__ == "__main__":
#     rospy.init_node("path_tracker")

#     tfBuffer = tf2_ros.Buffer()
#     tflistener = tf2_ros.TransformListener(tfBuffer)

#     pub_duty = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
#     pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     try:
#         planner = path()
        
#     except rospy.ROSInterruptException:
#         pass
    
#     rospy.spin()