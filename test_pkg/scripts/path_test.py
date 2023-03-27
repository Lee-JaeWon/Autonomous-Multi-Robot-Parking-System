#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
class Path_test():
    def __init__(self):
        rospy.init_node("path_node")
        rospy.Subscriber("/move_base/NavfnROS/plan",Path,self.callback)
        self.flag = False
        self.path = None
        rospy.spin()
    def callback(self,data):

        if self.flag == False:
            self.path = data.poses
            self.flag = True
        
        # print(self.path)
            print(len(self.path))
            for i in self.path:
                x = round(i.pose.position.x,3)
                y = round(i.pose.position.y ,3)
                print("x : {} y : {}".format(x,y))    
                time.sleep(0.125)    

if __name__ == "__main__":
    start = Path_test()
        