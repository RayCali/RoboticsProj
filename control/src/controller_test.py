#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from robp_msgs.msg import Encoders
from robp_msgs.msg import DutyCycles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_geometry_msgs
import tf_conversions
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros

#Listen to the aruco marker and publish the position of the marker in the map frame
def marker_callback(msg: MarkerArray):
    

# x = 0.0
# y = 0.0
# theta = 0.0

# def newOdom(msg: Odometry):
#     global x
#     global y
#     global theta

#     br = tf2_ros.TransformBroadcaster()
#     t = TransformStamped()

#     stamper = msg.header.stamp
#     timeout = rospy.Duration(0.5)
#     try:
#         trans = tfBuffer.lookup_transform('map','base_link', stamper)
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         return

#     pose_stamp = PoseStamped()
#     pose_stamp.pose.position.x = 2
#     pose_stamp.pose.position.y = 2
#     pose_stamp.pose.position.z = 0

#     pose_stamp.pose.orientation.x = 0
#     pose_stamp.pose.orientation.y = 0
#     pose_stamp.pose.orientation.z = 0
#     pose_stamp.pose.orientation.w = 0

#     t.header.stamp = msg.header.stamp
#     t.header.frame_id = "map"
#     t.child_frame_id = "destination"

#     transformed = tf2_geometry_msgs.do_transform_pose(pose_stamp,trans)
#     t.transform.translation.x = transformed.pose.position.x
#     t.transform.translation.y = transformed.pose.position.y
#     t.transform.translation.z = transformed.pose.position.z

#     t.transform.rotation.x = 0
#     t.transform.rotation.y = 0
#     t.transform.rotation.z = 0
#     t.transform.rotation.w = 0
#     br.sendTransform(t)

#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y

# sub = rospy.Subscriber('/odom', Odometry, newOdom)
# pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)
# rospy.init_node('speed_controller')

# speed = DutyCycles()

# r = rospy.Rate(4)

# goal = Point()
# goal.x = 2
# goal.y = 2



# while not rospy.is_shutdown():
#     inc_x = goal.x - x
#     inc_y = goal.y - y
#     angle_to_goal = math.atan2(inc_y, inc_x)
#     print(angle_to_goal)

#     if abs(angle_to_goal - theta) > 0.1:
#         speed.duty_cycle_left = 0.0
#         speed.duty_cycle_right = 0.3
#         # print("turning")
#     else:
#         speed.duty_cycle_left = 0.5
#         speed.duty_cycle_right = 0.0
#         print("moving forward")

#     pub.publish(speed)
#     r.sleep()



# #sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, marker)


# if __name__ == '__main__':
#     tfBuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfBuffer)
    
#     rospy.spin()