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
                    And([isNotExploring()]),
                    Or([
                        isPicked(),
                        isAtToy(),
                        And([
                            Or([
                                isPlanned(),
                                doPlanPathToy()
                                ]),
                            doMoveAlongPathToy()
                        ])
                    ]),
                    Or([
                        isPicked(),
                        doPickup()
                    ]),
                    Or([
                        isAtBox(),
                        And([
                            Or([
                                isPlannedBox(),
                                doPlanPathBox()
                            ]),
                            doMoveAlongPathBox(),
                        ])
                    ]),
                    Or([
                        isPlaced(),
                        doPlace()
                    ]),
                    doPlace(),
                    Or([
                        isNotPair(),
                        doReset()
                    ])
                    
                ])
            ]),
            isNotExploring(),
            Or([
                isSelected(),
                doSelect()

            ]),
            Or([
                isExplorationPathPlanned(),
                doPlanExplorationPath(),
            ]),
            doMoveAlongPathGlobal()
        ])
    )
    count = 0
    while not rospy.is_shutdown():
        rospy.init_node("behavior_tree")
        rospy.loginfo("BEHAVIOR TREE TICK#%d", count)
        count += 1
        root.tick()
        rospy.sleep(2)
