#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, PoseStamped, Pose, Twist
from robp_msgs.msg import Encoders
from aruco_msgs.msg import MarkerArray
from robp_msgs.msg import DutyCycles
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from msg_srv_pkg.msg import objectPoseStampedLst
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import tf
from msg_srv_pkg.srv import Request, RequestRequest, RequestResponse
from std_msgs.msg import Float64
from nav_msgs.msg import Path
SUCCESS, RUNNING, FAILURE = 1, 0, -1

class path(object):
    def __init__(self):
        # ROS Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pose = rospy.Subscriber("/detection/pose_baseLink", objectPoseStampedLst, self.distance_to_goal, queue_size=1)
        self.object_finalpose_pub = rospy.Publisher('/object_finalpose', PoseStamped, queue_size=10)
        # ROS Subscribers
        # self.goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.tracker, queue_size=1) # has to be the pose of the postion we want to go to
        self.s =rospy.ServiceProxy('/no_collision', Request)
        self.done_once = False
        self.rate = rospy.Rate(20)
        # self.covariance_sub = rospy.Subscriber("/radius", Float64, self.Radius, queue_size=1)
        self.radius_sub = float(1)
        self.objectpose = PoseStamped()
        self.listen_once = False
        self.updated_first_pose = PoseStamped()
        self.atToy_srv = rospy.Service("/atend", Request, self.arrivedAtEnd)

        self.movePath_srv = rospy.Service('/movePath', Request, self.doMovePathResponse)

        self.moveto_pub = rospy.Publisher('/Pathnodes', Path, queue_size=1)
        self.moveto_sub = rospy.Subscriber('/Pathnodes', Path, self.tracker, queue_size=1)
        
        self.STATE = FAILURE
        self.running = False
        self.objectpose = None
        self.path = None

        self.detection_sub = rospy.Subscriber("/revised", Path, self.doSavepath, queue_size=1)
    

    def doMovePathResponse(self, req: RequestRequest):
        if not self.running:
            if self.Path is None:
                return RequestResponse(FAILURE) 
            self.running = True
            self.moveto_pub.publish(self.Path)
            return RequestResponse(RUNNING)
        if self.running:
            if self.STATE == RUNNING:
                return RequestResponse(RUNNING)
            if self.STATE == FAILURE:
                self.running = False
                return RequestResponse(FAILURE)
            if self.STATE == SUCCESS:
                self.running = False
                return RequestResponse(SUCCESS)

    def arrivedAtEnd(self, req: Request):
        if self.STATE == SUCCESS:
            # self.STATE = FAILURE
            return RequestResponse(SUCCESS)
        return RequestResponse(FAILURE)
    
    def doSaveObjectpose(self, msg: Path):
        # if len(msg) == 0:
        #     rospy.loginfo("No object detected!!!!!!!!!!!!!!")
        #     exit()
        rospy.loginfo("Object detected")
        if self.Path is None or (rospy.Time.now().secs - self.Path.header.stamp.secs > 1):
            self.Path= msg


    # def Radius(self, msg:Float64):
    #     self.radius_sub = msg.data
    
    
    def tracker(self, msg: Path):
        self.STATE = RUNNING
        if not self.done_once:
            path = msg
            for point in path.poses:
                rospy.loginfo("moving to next node")
                rospy.sleep(1)
                self.twist = Twist()
                currentpose = PoseStamped()
                currentpose.pose = point.pose.position
                currentpose.header = point.header
            

                try:
                    trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout = rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from map frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
                while math.atan2(self.inc_y, self.inc_x)< -0.05: # or math.atan2(inc_y, inc_x) < -0.2:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -0.7
                    # rospy.loginfo("Turning right")
                    # rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()
                    try:
                        self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                        self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr("Failed to transform point from map frame to base_link frame")
                        pass
                    self.inc_x = self.goal_pose.pose.position.x
                    self.inc_y = self.goal_pose.pose.position.y

                while math.atan2(self.inc_y, self.inc_x) > 0.05: # or math.atan2(inc_y, inc_x) < -0.2:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.7
                    # rospy.loginfo("Turning left")
                    # rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()
                    try:
                        self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                        self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr("Failed to transform point from map frame to base_link frame")
                        pass
                    self.inc_x = self.goal_pose.pose.position.x
                    self.inc_y = self.goal_pose.pose.position.y
                
                self.twist.angular.z = 0.0
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                rospy.sleep(1)
                self.acceleration = 0.05
                self.deceleration = 0.05
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from map frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
                condition = math.sqrt(self.inc_x**2 + self.inc_y**2) > 0.05
                distance = math.sqrt(self.inc_x**2 + self.inc_y**2)
                latestupdate = rospy.Time.now()
                while distance > 0.02:
                    
                    rospy.loginfo("Waiting for service")
                    rospy.wait_for_service('/no_collision')
                    rospy.loginfo("Service found")
                    resp1 = self.s()
                    rospy.loginfo(resp1)
                    #resp1.wait_for_result()
                    if resp1.success is not True:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.pub_twist.publish(self.twist)
                        rospy.loginfo("Collision detected")
                        self.STATE = FAILURE
                    waitingtime = 1

                    if (rospy.Time.now().secs - latestupdate.secs) > 2:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.pub_twist.publish(self.twist)
                        rospy.sleep(1)
                        latestupdate = rospy.Time.now()
                        # now = rospy.Time.now()
                        # while((rospy.Time.now().secs-now.secs)<waitingtime):
                        #     continue
                        # latestupdate = rospy.Time.now()
                    try:
                            self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                            self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr("Failed to transform point from map frame to base_link frame")
                        pass
                    self.inc_x = self.goal_pose.pose.position.x
                    self.inc_y = self.goal_pose.pose.position.y
                    
                    if self.twist.linear.x < 0.6 and distance>self.twist.linear.x**2/(2*self.deceleration):
                        self.twist.linear.x += self.acceleration
                        # rospy.loginfo(self.twist.linear.x)

                    elif self.twist.linear.x >= 0.6 and distance>self.twist.linear.x**2/(2*self.deceleration): #eller acceleration
                        self.twist.linear.x = 0.6

                    else:
                        if self.twist.linear.x >0.15:
                            self.twist.linear.x -= self.deceleration
                        # rospy.loginfo("Decelerating")
                        # rospy.loginfo(self.twist.linear.x)
                
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()
                    try:
                        self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                        self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr("Failed to transform point from map frame to base_link frame")
                        pass
                    self.inc_x= self.goal_pose.pose.position.x
                    self.inc_y= self.goal_pose.pose.position.y
                    
                    if math.atan2(self.inc_y, self.inc_x) > 0.1:
                        self.twist.angular.z = 0.2 #either -0.2 or 0.2
                        self.pub_twist.publish(self.twist)
                        self.rate.sleep()

                    if math.atan2(self.inc_y, self.inc_x) < -0.1:
                        self.twist.angular.z = -0.2 #either -0.2 or 0.2
                        self.pub_twist.publish(self.twist)
                        self.rate.sleep()
                    distance = math.sqrt(self.inc_x**2 + self.inc_y**2)
                    rospy.loginfo("Distance to next node:")
                    rospy.loginfo(distance)
            self.done_once = True
        self.STATE = SUCCESS
        return
    
        # self.rate.sleep()
        # try:
        #     self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
        #     self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #         rospy.logerr("Failed to transform point from odom frame to base_link frame")
        #         pass
        # self.inc_x= self.goal_pose.pose.position.x
        # self.inc_y= self.goal_pose.pose.position.y
        # if math.sqrt(self.inc_x**2 + self.inc_y**2) < 0.05:
        #     self.done_once = True
        #     self.twist.linear.x = 0.0
        #     self.twist.linear.z = 0.0
        #     self.pub_twist.publish(self.twist)
        #     self.rate.sleep()
        #     try:
        #         self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
        #         self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             rospy.logerr("Failed to transform point from odom frame to base_link frame")
        #             pass
        #     self.inc_x= self.goal_pose.pose.position.x
        #     self.inc_y= self.goal_pose.pose.position.y
        #     # rospy.sleep(1)
        # else:
        #     rospy.loginfo('Going towards next goal')
        #     self.done_once = False

if __name__ == "__main__":
    rospy.init_node("path_follower")
    rospy.loginfo("Starting path_tracker node")
    # getCanIGetThereWithoutAnyCollisions = rospy.ServiceProxy('get_can_i_get_there_without_any_collisions', Node)
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
