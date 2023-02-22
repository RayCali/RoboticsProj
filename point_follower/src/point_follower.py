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


# done_once = False
# class path(object):
#     def __init__(self):
#         self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, tracker)
#         # self.robot = rospy.Subscriber("/tf", TFMessage, self.robot_pos)
        
# def tracker(msg:PoseStamped):
#     global done_once
#     if not done_once:
#         cam_pose = PoseStamped()
#         twist = Twist()
#         duty = DutyCycles()
#         rospy.sleep(2)

#         print('tracker hej')
#         # self.robot_pose = TFMessage()

#         # print("tracker function working")
#         rate = rospy.Rate(20)
#         msgTime = msg.header.stamp
#         # print("Hej!")
#         cam_pose.header.stamp = msg.header.stamp
#         cam_pose.header.frame_id = msg.header.frame_id
#         cam_pose.pose = msg.pose
        
#         # print(self.robot_pose.x, self.robot_pose.y)
#         # return self.robot_pose

#         try:
#             trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
#             goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
#             # print(self.goal_pose)

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             rospy.logerr("Failed to transform point from odom frame to base_link frame")
#             pass
        

#         # t = TransformStamped()

#         # self.robot_pose = msg.transforms[0].transform.translation
#         # self.robot_pose = msg.transforms[0].transform.rotation  do not use
#         # print(self.robot_pose)
        
#         # broadcaster = tf2_ros.TransformBroadcaster()
#         # t.header.frame_id = "map"
#         # t.child_frame_id = "goal"
#         # t.header.stamp = msgTime
#         # t.transform.translation = self.goal_pose.pose.position
#         # t.transform.rotation = self.goal_pose.pose.orientation
#         # print('tracker hej')

#         # print('before x and y')

#         # print(self.goal_pose)

#         distance_x = goal_pose.pose.position.x
#         distance_y = goal_pose.pose.position.y
#         rotation = goal_pose.pose.orientation.z
#         inc_x = distance_x
#         inc_y = distance_y
#         # print(inc_x, inc_y)

#         rospy.loginfo(abs(rotation))
#         # rospy.loginfo(math.atan2(inc_y, inc_x))
#         while abs(math.atan2(inc_y, inc_x)) > 0.1: # or math.atan2(inc_y, inc_x) < -0.2:
#             twist.linear.x = 0.0
#             twist.angular.z = 0.7

#             rospy.loginfo("Turning")
#             pub_twist.publish(twist)
#             rate.sleep()
#             try:
#                 trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
#                 goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 rospy.logerr("Failed to transform point from odom frame to base_link frame")
#                 pass
#             inc_x= goal_pose.pose.position.x
#             inc_y= goal_pose.pose.position.y
#             # rospy.loginfo(math.atan2(inc_y, inc_x))
        
#         twist.angular.z = 0.0
#         pub_twist.publish(twist)
#         rate.sleep()
#         rospy.sleep(1)
#         while math.sqrt(inc_x**2 + inc_y**2) > 0.1:
#             # rospy.loginfo(math.sqrt(inc_x**2 + inc_y**2))
#             twist.linear.x = 0.3
#             twist.angular.z = 0.0
#             pub_twist.publish(twist)
#             rate.sleep()
#             try:
#                 trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
#                 goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 rospy.logerr("Failed to transform point from odom frame to base_link frame")
#                 pass
#             inc_x= goal_pose.pose.position.x
#             inc_y= goal_pose.pose.position.y
#             if (math.atan2(inc_y, inc_x)) > 0.1:
#                 twist.linear.x = 0.3
#                 twist.angular.z = -0.2 #either -0.2 or 0.2
#                 pub_duty.publish(twist)
#                 rate.sleep()

#             elif (math.atan2(inc_y, inc_x)) < -0.1:
#                 twist.linear.x = 0.3
#                 twist.angular.z = 0.2 #either -0.2 or 0.2
#                 pub_duty.publish(twist)
#                 rate.sleep()
        
#         rospy.loginfo('You have reached the goal')
#         done_once = True
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         pub_twist.publish(twist)
#         rate.sleep()
#             # broadcaster.sendTransform(t)


#             # rate.sleep()
# if __name__ == "__main__":
#     rospy.init_node("path_tracker")
#     rospy.loginfo("Starting path_tracker node")

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


done_once = False
class path(object):
    def __init__(self):
        self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, tracker)
        # self.robot = rospy.Subscriber("/tf", TFMessage, self.robot_pos)
        
def tracker(msg:PoseStamped):
    global done_once
    if not done_once:
        cam_pose = PoseStamped()
        twist = Twist()
        rospy.sleep(2)

        print('tracker hej')
        # self.robot_pose = TFMessage()

        # print("tracker function working")
        rate = rospy.Rate(20)
        msgTime = msg.header.stamp
        # print("Hej!")
        cam_pose.header.stamp = msg.header.stamp
        cam_pose.header.frame_id = msg.header.frame_id
        cam_pose.pose = msg.pose
        
        # print(self.robot_pose.x, self.robot_pose.y)
        # return self.robot_pose

        try:
            trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
            goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
            # print(self.goal_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform point from odom frame to base_link frame")
            pass

        distance_x = goal_pose.pose.position.x
        distance_y = goal_pose.pose.position.y
        rotation = goal_pose.pose.orientation.z
        inc_x = distance_x
        inc_y = distance_y
        # print(inc_x, inc_y)

        rospy.loginfo(abs(rotation))
        # rospy.loginfo(math.atan2(inc_y, inc_x))
        while math.atan2(inc_y, inc_x)< -0.08: # or math.atan2(inc_y, inc_x) < -0.2:
            twist.linear.x = 0.0
            twist.angular.z = -0.7

            rospy.loginfo("Turning right")
            pub_twist.publish(twist)
            rate.sleep()
            try:
                trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
                goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            inc_x= goal_pose.pose.position.x
            inc_y= goal_pose.pose.position.y

        while math.atan2(inc_y, inc_x) > 0.08: # or math.atan2(inc_y, inc_x) < -0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.7

            rospy.loginfo("Turning left")
            pub_twist.publish(twist)
            rate.sleep()
            try:
                trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
                goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            inc_x= goal_pose.pose.position.x
            inc_y= goal_pose.pose.position.y
        
        twist.angular.z = 0.0
        pub_twist.publish(twist)
        rate.sleep()
        rospy.sleep(1)
        acceleration = 0.01
        deceleration = 0.08
        while math.sqrt(inc_x**2 + inc_y**2) > 0.08:
            try:
                trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
                goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            inc_x= goal_pose.pose.position.x
            inc_y= goal_pose.pose.position.y
            if twist.linear.x < 0.6 and math.sqrt(inc_x**2 + inc_y**2)>twist.linear.x**2/(2*deceleration):
                twist.linear.x += acceleration
                rospy.loginfo(twist.linear.x)
                # pub_twist.publish(twist)
                # rate.sleep()
                
            elif twist.linear.x >= 0.6 and math.sqrt(inc_x**2 + inc_y**2)>twist.linear.x**2/(2*deceleration): #eller acceleration
                twist.linear.x = 0.6
                # pub_twist.publish(twist)
                # rate.sleep()
            
            else:# math.sqrt(inc_x**2 + inc_y**2)<=twist.linear.x**2/(2*acceleration):
                twist.linear.x -= deceleration
                rospy.loginfo("Decelerating")
                rospy.loginfo(twist.linear.x)
                # pub_twist.publish(twist)
                # rate.sleep()
            # rospy.loginfo(math.sqrt(inc_x**2 + inc_y**2))
            
            pub_twist.publish(twist)
            rate.sleep()
            try:
                trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
                goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            inc_x= goal_pose.pose.position.x
            inc_y= goal_pose.pose.position.y
            if math.atan2(inc_y, inc_x) > 0.08:
                twist.angular.z = 0.1 #either -0.2 or 0.2
                pub_twist.publish(twist)
                rate.sleep()
            try:
                trans = tfBuffer.lookup_transform("base_link", "odom", msgTime, timeout=rospy.Duration(2.0))
                goal_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            inc_x= goal_pose.pose.position.x
            inc_y= goal_pose.pose.position.y

            if math.atan2(inc_y, inc_x) < -0.08:
                twist.angular.z = -0.1 #either -0.2 or 0.2
                pub_twist.publish(twist)
                rate.sleep()

        rospy.loginfo('You have reached the goal')
        done_once = True
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub_twist.publish(twist)
        rate.sleep()
            # broadcaster.sendTransform(t)


            # rate.sleep()
if __name__ == "__main__":
    rospy.init_node("point_follower")
    rospy.loginfo("Starting path_tracker node")

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)

    pub_duty = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    try:
        path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
