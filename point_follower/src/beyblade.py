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
SUCCESS, RUNNING, FAILURE = 1, 0, -1

class path(object):
    def __init__(self):

        self.moveToToy_srv = rospy.Service("/srv/spin/beyblade/brain", Request, self.doMoveToToyResponse)
        self.running=False
        self.STATE = FAILURE
        self.twist = Twist()
        self.spinpub = rospy.Publisher("/spin", Twist, queue_size=1)
        self.spinsub = rospy.Subscriber("/spin", Twist, self.spin)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    

    def doMoveToToyResponse(self, req: RequestRequest):
        if not self.running:
            self.running = True
            self.spinpub.publish(self.twist)
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
    def spin(self, msg):
        rospy.loginfo("im in")
        base_link = tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
        anglelist = tf.transformations.euler_from_quaternion([base_link.transform.rotation.x, base_link.transform.rotation.y, base_link.transform.rotation.z, base_link.transform.rotation.w])
        currentyaw = anglelist[2]
        latesttime = rospy.Time.now()
        condition = np.abs(currentyaw - anglelist[2]) < 5
        switch = False
        while condition:
            rospy.loginfo(currentyaw - anglelist[2])
            if np.abs(currentyaw - anglelist[2]) > 3:
                switch = True
            if switch:    
                condition = np.abs(currentyaw - anglelist[2]) > 0.1
            else:
                condition = np.abs(currentyaw - anglelist[2]) < 5
            self.twist.angular.z = 0.7
            self.pub_twist.publish(self.twist)
            rospy.Rate(20).sleep()
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
        return
 
if __name__ == "__main__":
    rospy.init_node("beyblade")
    rospy.loginfo("Starting beyblade node")
    # getCanIGetThereWithoutAnyCollisions = rospy.ServiceProxy('get_can_i_get_there_without_any_collisions', Node)
    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)
    try:
        follower = path()
        
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
