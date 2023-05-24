#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import OccupancyGrid,Odometry
import lidar_to_grid_map
import tf
import numpy as np
from math import *
'''
angle_min: 0.0
angle_max: 6.28318977355957
angle_increment: 0.017501922324299812
time_increment: 0.0
scan_time: 0.0
range_min: 0.11999999731779099
range_max: 3.5
ranges:
'''
class scan_test:

    def __init__(self):
        rospy.init_node("laser_test_node")
        rospy.Subscriber("/scan",LaserScan,self.ScanCB)
        rospy.Subscriber("/map",OccupancyGrid,self.callback)
        rospy.Subscriber('/odom',Odometry,self.OdomCB)
        self.increment = 0
        rospy.spin()
        
    def ScanCB(self,data):
        print(len(data.ranges))
        self.increment = data.angle_increment
        for val, idx in enumerate(data.ranges):
            x = val * cos(idx * self.increment)
            y = val * sin(idx * self.increment)
        
if __name__ =="__main__":
    start = scan_test()
    