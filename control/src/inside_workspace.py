#!/usr/bin/python3
from math import pi
import rospy
from robp_msgs.msg import DutyCycles
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from robp_msgs.msg import Encoders 
from visualization_msgs.msg import Marker
import tf_conversions
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import tf
#create a point class
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

workspace = None
twistmsg = None
listener = None
tf_buffer = None
pub= None
def workspacecallback(data:Marker):
    global workspace
    workspace = data.points[:-1]

def twistcallback(data:Twist):
    global twistmsg, listener, workspace, tf_buffer, pub
    rospy.loginfo("got twist")
    twistmsg = data
    if twistmsg.linear.x != 0:
        transform = tf_buffer.lookup_transform('arucomap', 'base_link', rospy.Time(0))
        #poly = [Point(0,0), Point(-1.3,2), Point(1.3,1), Point(3.1,-0.5)]
        poly = workspace
        if point_inside_polygon(transform.transform.translation.x+twistmsg.linear.x, transform.transform.translation.y, poly):
            rospy.loginfo("inside workspace")
            pub.publish(twistmsg)
        else:
            rospy.loginfo("outside workspace")
            twistmsg.linear.x = 0
            pub.publish(twistmsg)
    else:
        pub.publish(twistmsg)
            


def point_inside_polygon(x,y,poly):
    n = len(poly)
    inside =False
    p1x,p1y = poly[0].x,poly[0].y
    for i in range(n+1):
        p2x,p2y = poly[i % n].x,poly[i % n].y
        print(i%n)
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y
    return inside

if __name__ == '__main__':
    #determine if a point is inside a polygon or not
    #Polygon is a list of (x,y) pairs.
    #calculate the distance to the closest line segment
    rospy.init_node('inside_workspace')
    vel_sub = rospy.Subscriber('/cmd_vel', Twist, twistcallback)
    workspace_sub = rospy.Subscriber('/boundaries', Marker, workspacecallback)
    pub = rospy.Publisher('/cmd_vel_proc', Twist, queue_size=1)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    #create a polygon
    rospy.spin()

    #check if a point is inside the polygon
    #print(point_inside_polygon(1.5,0.5,poly))