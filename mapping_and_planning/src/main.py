#!/usr/bin/python3

import rospy
from gridmapping import Map
from provider import PathProvider
from global_explorer import getMostValuedCell 

if __name__ == "__main__":

    rospy.init_node("mapping_and_planning")
        
    m = Map(True, 11, 11, 0.05)
    # p = PathProvider(m)
    rospy.sleep(1)
    # explorer_info = getMostValuedCell(m.matrix, m.grid.info.width, m.grid.info.height)
    m.doPublish()
    while not rospy.is_shutdown():
        rospy.sleep(0.05)
        m.doPublish()
        
