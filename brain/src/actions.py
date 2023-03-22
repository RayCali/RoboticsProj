#!/usr/bin/python3

from superclasses import *
import rospy
from std_srvs.srv import SetBool

class doExplore(Action):
    def __init__(self) -> None:
        super().__init__()
        self.service = rospy.ServiceProxy("Explore/src", SetBool)
