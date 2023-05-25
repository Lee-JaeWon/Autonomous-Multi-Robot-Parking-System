#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
class lidar_test():
    def __init__(self):
        rospy.init_node("lidar_node")
        rospy.Subscriber("/robot_1/scan",LaserScan,self.callback)
        rospy.spin()
    def callback(self,data):
        '''
        sensor_msgs/LaserScan 
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        float32 angle_min
        float32 angle_max
        float32 angle_increment
        float32 time_increment
        float32 scan_time
        float32 range_min
        float32 range_max
        float32[] ranges
        float32[] intensities
        '''
        max_ = 0
        min_ = 10900
        cnt = 0
        print(data)
        # for idx,value in enumerate(data.ranges):

        #     if (value < 0.185 and value > 0.165 ):
        
        #         r = (data.ranges[idx-1] + data.ranges[idx+1]) / 2.0
        #         print(r)
        
        #     if(0.15 <= value <= 0.2):
        #         cnt += 1 

        # #         max_ = max(max_,value)
        # #         min_ = min(min_,value)
        # #         # print("idx : {} value : {}".format(idx,value))
        # # print(min_ , max_)
        # print(cnt)
if __name__ == "__main__":

    start = lidar_test()
        