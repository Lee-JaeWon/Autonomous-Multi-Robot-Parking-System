#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import time,math

class goal_test():

    def __init__(self):
        rospy.init_node("path_node")
        rospy.Subscriber("/odom",Odometry,self.callback)
        self.msgpub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.Goal)
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.path_lst = [(-1.5 , - 0.5 ),(-1.0 , - 0.5 ),(-0.5 , - 0.5 ),(0.0 , - 0.5 ),(1.0 , - 0.5 )]
        self.path_Num = 0
        rospy.spin()
    def Goal(self,event):
        msg = PoseStamped()
        msg.header.frame_id="map"
        try:
            msg.pose.position.x = self.path_lst[self.path_Num][0]
            msg.pose.position.y = self.path_lst[self.path_Num][1]
        except:
            msg.pose.position.x = self.path_lst[-1][0]
            msg.pose.position.y = self.path_lst[-1][1]
        # msg.pose.orientation = self.orientation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0


        rospy.loginfo("local_goal pub")
        self.msgpub.publish(msg)

    def distance(self,A,B):
        dx = abs(A[0] - B[0])
        dy = abs(A[1] - B[1])
        return math.sqrt(dx ** 2 + dy ** 2)
        # return distance    


    def callback(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.orientation = data.pose.pose.orientation
        try:
            dist = self.distance((self.path_lst[self.path_Num]),(self.x,self.y))
        except:
            dist = self.distance((self.path_lst[-1]),(self.x,self.y))

        if (dist <= 0.4): self.path_Num += 1
        # print(self.x,self.y)


if __name__ == "__main__":
    start = goal_test()
        
