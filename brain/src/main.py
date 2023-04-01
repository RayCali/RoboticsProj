#!/usr/bin/python3

import numpy
import superclasses
from conditions import *
from actions import * 
from utilities import Root, And, Or

if __name__=="__main__":
    root = Root(
        And([
            Or([
                isLocalized(),
                doLocalize()
            ]),
            Or([
                isNotPair(),
                And([
                    Or([
                        isPicked(),
                        isInFrontToy(),
                    ])
                ])
            ])

        ])
    )
    while not rospy.is_shutdown():
        rospy.init_node("behavior_tree")
        root.tick()
        rospy.sleep(5)
