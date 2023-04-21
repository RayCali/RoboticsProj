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
from msg_srv_pkg.srv import Node, Moveto, MovetoResponse, MovetoRequest, NoCollision, NoCollisionRequest, NoCollisionResponse, Request, RequestResponse
from std_msgs.msg import Float64
SUCCESS, RUNNING, FAILURE = 1, 0, -1

class path(object):
    def __init__(self):
        # ROS Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pose = rospy.Subscriber("/detection/pose_baseLink", objectPoseStampedLst, self.distance_to_goal, queue_size=1)
        self.object_finalpose = rospy.Publisher('/object_finalpose', PoseStamped, queue_size=10)
        # ROS Subscribers
        # self.goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.tracker, queue_size=1) # has to be the pose of the postion we want to go to
        self.s =rospy.ServiceProxy('/no_collision', NoCollision)
        self.done_once = False
        self.rate = rospy.Rate(20)
        # self.covariance_sub = rospy.Subscriber("/radius", Float64, self.Radius, queue_size=1)
        self.radius_sub = float(1)
        self.objectpose = PoseStamped()
        self.listen_once = False
        self.updated_first_pose = PoseStamped()
        self.atToy_srv = rospy.Service("/atToy", Request, self.arrivedAtToy)

        self.moveToToy_srv = rospy.Service('/moveToToy', Moveto, self.doMoveToToyResponse)

        self.moveto_pub = rospy.Publisher('/toyPose', PoseStamped, queue_size=1)
        self.moveto_sub = rospy.Subscriber('/toyPose', PoseStamped, self.tracker, queue_size=1)
        
        self.STATE = FAILURE
        self.running = False

    def doMoveToToyResponse(self, req: Moveto):
        if not self.running:
            self.running = True
            self.moveto_pub(Moveto.goal)
            return MovetoResponse(RUNNING)
        if self.running:
            if self.STATE == RUNNING:
                return MovetoResponse(RUNNING)
            if self.STATE == FAILURE:
                self.running = False
                return MovetoResponse(FAILURE)
            if self.STATE == SUCCESS:
                self.running = False
                return MovetoResponse(SUCCESS)

    def arrivedAtToy(self, req: Request):
        if self.STATE == SUCCESS:
            self.STATE = RUNNING
            return RequestResponse(SUCCESS)
        
        return RequestResponse(FAILURE)


    # def Radius(self, msg:Float64):
    #     self.radius_sub = msg.data
    def distance_to_goal(self,msg):
        #self.distance_to_object = math.sqrt(msg.pose.position.x**2 + msg.pose.position.y**2)
        i = 50
        if self.listen_once:
            if len(msg.PoseStamped) == 0:
                rospy.loginfo("No object detected")
                exit()
            self.objectpose = msg.PoseStamped[0]
            if self.objectpose.pose.position.x < 0.15 or self.objectpose is None or self.objectpose.pose.position.x > 0.4:
                self.listen_once = True
            else: 
                self.listen_once = False
                trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
                self.updated_first_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose,trans)
            if i>0:
                rospy.loginfo("Distance to object: {}".format(self.objectpose.pose.position.x))
                i = i-1
            #rospy.loginfo("Label: {}".format(msg.object_class[0]))
    
    
    
    
    
    
    
    
    def tracker(self, msg: PoseStamped):
        if not self.done_once:
            self.cam_pose = msg
            self.twist = Twist()
            rospy.sleep(2)
            # if len(msg.PoseStamped) == 0:
            #     rospy.loginfo("No object detected")
            #     exit()
            
            # self.rate = rospy.Rate(20)
            # rospy.Time(0) = msg.header.stamp
            # rospy.Time(0) = msg.header.stamp
            # self.cam_pose.header.frame_id = msg.header.frame_id
            # self.cam_pose.pose = msg.pose
            try:
                self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass

            self.rotation = self.goal_pose.pose.orientation.z
            self.inc_x = self.goal_pose.pose.position.x
            self.inc_y = self.goal_pose.pose.position.y

            # rospy.loginfo(abs(rotation))
            while math.atan2(self.inc_y, self.inc_x)< -0.05: # or math.atan2(inc_y, inc_x) < -0.2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.7
                # rospy.loginfo("Turning right")
                # rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
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
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
            
            self.twist.angular.z = 0.0
            self.pub_twist.publish(self.twist)
            self.rate.sleep()
            rospy.sleep(1)
            self.acceleration = 0.01
            self.deceleration = 0.05
            try:
                self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            self.inc_x = self.goal_pose.pose.position.x
            self.inc_y = self.goal_pose.pose.position.y
            condition = math.sqrt(self.inc_x**2 + self.inc_y**2) > 0.05
            distance = math.sqrt(self.inc_x**2 + self.inc_y**2)
            switch =True
            latest_pose=None
            
            while condition:
                # rospy.loginfo("Waiting for service")
                # rospy.wait_for_service('/no_collision')
                # rospy.loginfo("Service found")
                # resp1 = self.s()
                # rospy.loginfo(resp1)
                # #resp1.wait_for_result()
                # if resp1.success is not True:
                #     self.twist.linear.x = 0.0
                #     self.twist.angular.z = 0.0
                #     self.pub_twist.publish(self.twist)
                #     rospy.loginfo("Collision detected")
                #     return MovetoResponse(False,"Failure")
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
                if switch:
                    if self.twist.linear.x < 0.6 and distance>self.twist.linear.x**2/(2*self.deceleration):
                        self.twist.linear.x += self.acceleration
                        # rospy.loginfo(self.twist.linear.x)

                    elif self.twist.linear.x >= 0.6 and distance>self.twist.linear.x**2/(2*self.deceleration): #eller acceleration
                        self.twist.linear.x = 0.6

                    else:
                        self.twist.linear.x -= self.deceleration
                        # rospy.loginfo("Decelerating")
                        # rospy.loginfo(self.twist.linear.x)
                else:
                    self.twist.linear.x = 0.15

                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x= self.goal_pose.pose.position.x
                self.inc_y= self.goal_pose.pose.position.y
                if math.atan2(self.inc_y, self.inc_x) > 0.05:
                    self.twist.angular.z = 0.2 #either -0.2 or 0.2
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()

                if math.atan2(self.inc_y, self.inc_x) < -0.05:
                    self.twist.angular.z = -0.2 #either -0.2 or 0.2
                    self.pub_twist.publish(self.twist)
                    self.rate.sleep()
                if switch is False:
                    current_trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    latest_pose=tf2_geometry_msgs.do_transform_pose(latest_pose,current_trans)
                    
                    rospy.loginfo("distance to obj straight")
                    rospy.loginfo(self.objectpose.pose.position.x + latest_pose.pose.position.x)
                    self.objectpose.pose.position.x = self.objectpose.pose.position.x + latest_pose.pose.position.x
                    self.objectpose.pose.position.y = self.objectpose.pose.position.y + latest_pose.pose.position.y
                    distance = math.sqrt(self.objectpose.pose.position.x**2 + self.objectpose.pose.position.y**2)
                    trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2.0))
                    latest_pose = PoseStamped()
                    latest_pose.header.frame_id = "map"
                    latest_pose.pose.position.x = trans.transform.translation.x
                    latest_pose.pose.position.y = trans.transform.translation.y
                    condition = distance > 0.1

                    
                else: 
                    distance = math.sqrt(self.inc_x**2 + self.inc_y**2)
                rospy.loginfo("Distance:")
                rospy.loginfo(distance)
                if math.sqrt(self.inc_x**2 + self.inc_y**2) < 0.3 and switch:
                    self.listen_once=True
                    while self.listen_once:
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.pub_twist.publish(self.twist)
                    
                    condition = distance > 0.1
                    switch = False
                    trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2.0))
                    latest_pose = PoseStamped()
                    latest_pose.header.frame_id = "map"
                    latest_pose.pose.position.x = trans.transform.translation.x
                    latest_pose.pose.position.y = trans.transform.translation.y
                    rospy.loginfo("Switch!!!!!!!!!!!!")
            rospy.loginfo("Out")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0 #I added these two lines just incase
            self.pub_twist.publish(self.twist)
            trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), timeout=rospy.Duration(2.0))
            final_pose_map = tf2_geometry_msgs.do_transform_pose(self.objectpose,trans)
            while math.atan2(self.inc_y, self.inc_x)< -0.005: # or math.atan2(inc_y, inc_x) < -0.2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.7
                #rospy.loginfo("Turning right")
                #rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y

            while math.atan2(self.inc_y, self.inc_x) > 0.005: # or math.atan2(inc_y, inc_x) < -0.2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.7
                #rospy.loginfo("Turning left")
                #rospy.loginfo(math.atan2(self.inc_y, self.inc_x))
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.cam_pose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
            self.done_once = True 
        rospy.loginfo('You have reached the goal')
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub_twist.publish(self.twist)
        self.distance_to_object = 500
        trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
        final_pose = tf2_geometry_msgs.do_transform_pose(final_pose_map,trans)
        self.object_finalpose.publish(final_pose)
        rospy.loginfo(self.objectpose)
        self.STATE = SUCCESS
    
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
    rospy.init_node("point_follower")
    rospy.loginfo("Starting path_tracker node")
    getCanIGetThereWithoutAnyCollisions = rospy.ServiceProxy('get_can_i_get_there_without_any_collisions', Node)
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
