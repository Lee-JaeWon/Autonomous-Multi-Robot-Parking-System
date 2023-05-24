#!/usr/bin/env python3

import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
class costmap:
    def __init__(self):
        rospy.init_node("cost_map")
        rospy.Subscriber("/move_base/local_costmap/costmap",OccupancyGrid,self.mapcallback)
        rospy.spin()
        
    def mapcallback(self,data):
        
        self.height = data.info.height
        self.width = data.info.width
        self.resolution = data.info.resolution
        
        temp = list(data.data)
        arr = np.array([temp])
        self.map = arr.reshape(self.height,self.width)

        for i in self.map:
            print(*i)
if __name__ =="__main__":
    start = costmap()
    