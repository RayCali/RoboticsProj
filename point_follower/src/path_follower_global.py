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
        self.tfbroadcaster = tf2_ros.TransformBroadcaster()
        # ROS Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.goal_pose = rospy.Subscriber("/detection/pose_baseLink", objectPoseStampedLst, self.distance_to_goal, queue_size=1)
        self.object_finalpose_pub = rospy.Publisher('/object_finalpose', PoseStamped, queue_size=10)
        # ROS Subscribers
        # self.goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.tracker, queue_size=1) # has to be the pose of the postion we want to go to
        self.collision_srv = rospy.ServiceProxy('/srv/no_collision/mapping_and_planning/path_follower', Request)
        self.publish_node = rospy.Publisher('/path_follower/node', Float64, queue_size=1)
        # self.covariance_sub = rospy.Subscriber("/radius", Float64, self.Radius, queue_size=1)


        self.updated_first_pose = PoseStamped()
        self.atToy_srv = rospy.Service("/atend", Request, self.arrivedAtEnd)
        self.explore = True
        self.movePath_srv = rospy.Service('/srv/doMoveAlongPathGlobal/path_follower_global/brain', Request, self.doMovePathResponse)

        self.moveto_pub = rospy.Publisher('/path_follower/tracker', Path, queue_size=1)
        self.moveto_sub = rospy.Subscriber('/path_follower/tracker', Path, self.tracker, queue_size=1)
        self.save_sub   = rospy.Subscriber('/rewired', Path, self.doSaveObjectpose, queue_size=1)
       

        #self.detection_sub = rospy.Subscriber("/revised", Path, self.doSavepath, queue_size=1)
    

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
        rospy.loginfo("Object detected")
        if self.Path is None:
            self.Path= msg


    # def Radius(self, msg:Float64):
    #     self.radius_sub = msg.data
    
    
    def tracker(self, msg: Path):
        self.STATE = RUNNING
        if not self.done_once:
            path = msg
            path.poses = path.poses[1:]
            rospy.loginfo("My path is: ")
            rospy.loginfo(msg)
            node_nr = Float64()
            node_nr.data = -1
            for point in path.poses:
                node_nr.data += 1
                to_log = "moving to next node" + str(np.round(point.pose.position.x,3)) + " " + str(np.round(point.pose.position.y,3))  
                self.publish_node.publish(node_nr)
                rospy.loginfo(to_log)
                rospy.loginfo(point)

                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "next_node"
                t.transform.translation.x = point.pose.position.x
                t.transform.translation.y = point.pose.position.y
                t.transform.rotation.w = 1
                self.tfbroadcaster.sendTransform(t)

                rospy.sleep(1)
                self.twist = Twist()
                currentpose = PoseStamped()
                currentpose.pose.position = point.pose.position
                currentpose.header = point.header
            

                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout = rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(currentpose, self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from map frame to base_link frame")
                    pass

                self.rotation = self.goal_pose.pose.orientation.z
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
                while distance > 0.03:
                    
                    rospy.loginfo("Waiting for service")
                    rospy.wait_for_service('/srv/no_collision/mapping_and_planning/path_follower')
                    rospy.loginfo("Service found")
                    res = self.collision_srv()
                    rospy.loginfo(res)
                    rospy.loginfo(res.success == False)
                    if res.success != SUCCESS:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.pub_twist.publish(self.twist)
                        rospy.loginfo("Collision detected")
                        self.STATE = FAILURE
                        return
                    #rospy.sleep(1)
                        
                    if (rospy.Time.now().secs - latestupdate.secs) > 1:
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.pub_twist.publish(self.twist)
                        rospy.sleep(1)
                        latestupdate = rospy.Time.now()
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
                    
                    if math.atan2(self.inc_y, self.inc_x) > 0.05:
                        self.twist.angular.z = 0.2 #either -0.2 or 0.2
                        self.pub_twist.publish(self.twist)
                        self.rate.sleep()

                    if math.atan2(self.inc_y, self.inc_x) < -0.05:
                        self.twist.angular.z = -0.2 #either -0.2 or 0.2
                        self.pub_twist.publish(self.twist)
                        self.rate.sleep()
                    distance = math.sqrt(self.inc_x**2 + self.inc_y**2)
                    to_log = "Distance to next node: " + str(np.round(distance, 3))
                    rospy.loginfo(to_log)
            self.done_once = True
        
        #spin base_link if exploring
        if self.explore:
            base_link = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
            anglelist = tf.transformations.euler_from_quaternion([base_link.transform.rotation.x, base_link.transform.rotation.y, base_link.transform.rotation.z, base_link.transform.rotation.w])
            currentyaw = anglelist[2]
            latesttime = rospy.Time.now()
            condition = np.abs(currentyaw - anglelist[2]) < 5
            switch = False
            while condition:
                rospy.loginfo(currentyaw - anglelist[2])
                if np(currentyaw - anglelist[2]) > 3:
                    switch = True
                if switch:    
                    condition = np.abs(currentyaw - anglelist[2]) > 0.1
                else:
                    condition = np.abs(currentyaw - anglelist[2]) < 5
                self.twist.angular.z = 0.7
                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    base_link = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
                    roll,pitch,currentyaw = tf.transformations.euler_from_quaternion([base_link.transform.rotation.x, base_link.transform.rotation.y, base_link.transform.rotation.z, base_link.transform.rotation.w])
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from map frame to base_link frame")
                    pass
                if (rospy.Time.now().secs - latesttime.secs) > 1:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.pub_twist.publish(self.twist)
                    rospy.sleep(1)
                    latesttime = rospy.Time.now()
        self.STATE = SUCCESS
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub_twist.publish(self.twist)
        self.done_once = False
        self.rate = rospy.Rate(20)
        self.STATE = FAILURE
        self.running = False
        self.Path = None
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
    rospy.init_node("path_follower_global")
    rospy.loginfo("Starting path_tracker node")
    # getCanIGetThereWithoutAnyCollisions = rospy.ServiceProxy('get_can_i_get_there_without_any_collisions', Node)
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
