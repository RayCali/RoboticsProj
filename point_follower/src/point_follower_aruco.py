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
from visualization_msgs.msg import Marker
SUCCESS, RUNNING, FAILURE = 1, 0, -1
from playsound import playsound

class path(object):
    def __init__(self):
        # ROS Publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.object_finalpose_pub = rospy.Publisher('/object_finalpose', PoseStamped, queue_size=10)
        self.aruco_sub      = rospy.Subscriber("/aruco_all/aruco/markers", MarkerArray, self.update, queue_size=1)
        # ROS Subscribers
        # self.goal = rospy.Subscriber("/detection/pose", objectPoseStampedLst, self.tracker, queue_size=1) # has to be the pose of the postion we want to go to
        self.s =rospy.ServiceProxy('/no_collision', Request)
        self.updated_first_pose = PoseStamped()
    

        self.moveToToy_srv = rospy.Service("/srv/doMoveToGoal/point_follower_aruco/path_follower_local", Request, self.doMoveToToyResponse)
        # self.covariance_sub = rospy.Subscriber("/radius", Float64, self.Radius, queue_size=1)
        #self.moveto_pub = rospy.Publisher('/toyPose', PoseStamped, queue_size=1)
        #self.moveto_sub = rospy.Subscriber('/toyPose', PoseStamped, self.tracker, queue_size=1)
        self.detection_sub = rospy.Subscriber("/targetPoseMap", objectPoseStampedLst, self.doSaveObjectpose, queue_size=1)
        self.done_once = False
        self.rate = rospy.Rate(20)
        self.objectpose = None
        self.listen_once = False
        self.goal_name = "lmao"
        self.STATE = FAILURE
        self.running = False
        self.objectpose = None
        self.objectpose_map = None
        self.arrrived = False
        self.updating = False
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
    

    def doMoveToToyResponse(self, req: RequestRequest):
        if self.objectpose_map is None:
            return RequestResponse(FAILURE)
        self.updating = True 
        rospy.loginfo("calling pointfollower_aruco tracker")
        self.tracker(self.objectpose_map.PoseStamped[0])
        return RequestResponse(self.STATE)
        #self.moveto_pub.publish(self.objectpose_map.PoseStamped[0])


   
    
    def doSaveObjectpose(self, msg: objectPoseStampedLst):
        # if len(msg) == 0:
        #     rospy.loginfo("No object detected!!!!!!!!!!!!!!")
        #     exit()
        #rospy.loginfo("Object detected")
        if "Box" in msg.object_class[0]:
            self.objectpose_map = msg


    # def Radius(self, msg:Float64):
    #     self.radius_sub = msg.data
   
    def update(self, msg:MarkerArray):
        if self.objectpose_map is None:
            pass
        else:    
            for marker in msg.markers:
                if marker.id == int(self.objectpose_map.object_class[0][-1]):
                    boxpose = PoseStamped()
                    boxpose.header.frame_id = "camera_link"
                    boxpose.pose = marker.pose.pose
                    transform = self.tf_buffer.lookup_transform('map', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
                    boxPose_map = tf2_geometry_msgs.do_transform_pose(boxpose, transform)
                    self.objectpose_map.PoseStamped[0]=boxPose_map



    
    def tracker(self, msg: PoseStamped):
        rospy.loginfo("MOVE TO BOX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # playsound('/home/robot/dd2419_ws/src/speaker/src/MoveToToy.mp3')
        self.STATE = RUNNING
        if not self.done_once:
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
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose_map.PoseStamped[0], self.trans)
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
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose_map.PoseStamped[0], self.trans)
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
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose_map.PoseStamped[0], self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
            
            self.twist.angular.z = 0.0
            self.pub_twist.publish(self.twist)
            self.rate.sleep()
           
            self.updating = False
            self.acceleration = 0.05
            self.deceleration = 0.05
            try:
                self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose_map.PoseStamped[0], self.trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to transform point from odom frame to base_link frame")
                pass
            self.inc_x = self.goal_pose.pose.position.x
            self.inc_y = self.goal_pose.pose.position.y
            while math.sqrt(self.inc_x**2 + self.inc_y**2) > 0.08:
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
                #     self.STATE = FAILURE
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose_map.PoseStamped[0], self.trans)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to transform point from odom frame to base_link frame")
                    pass
                self.inc_x = self.goal_pose.pose.position.x
                self.inc_y = self.goal_pose.pose.position.y
            
                self.twist.linear.x = 0.15

                self.pub_twist.publish(self.twist)
                self.rate.sleep()
                try:
                    self.trans = tfBuffer.lookup_transform("base_link", "map", rospy.Time(0), timeout=rospy.Duration(2.0))
                    self.goal_pose = tf2_geometry_msgs.do_transform_pose(self.objectpose_map.PoseStamped[0], self.trans)
                    
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
               
                    
            rospy.loginfo("Out")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0 #I added these two lines just incase
            self.pub_twist.publish(self.twist)

            self.done_once = True 
        rospy.loginfo('You have reached the goal')
        
        self.object_finalpose_pub.publish(PoseStamped())
        rospy.loginfo(self.objectpose)
        self.STATE = SUCCESS
        self.done_once = False
        self.objectpose = None
        self.listen_once = False
        self.goal_name = "lmao"
        self.running = False
        self.objectpose = None
        self.objectpose_map = None
        self.updating =False
        return
    
       

if __name__ == "__main__":
    rospy.init_node("point_follower_aruco")
    rospy.loginfo("Starting path_tracker node")
    # getCanIGetThereWithoutAnyCollisions = rospy.ServiceProxy('get_can_i_get_there_without_any_collisions', Node)
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
