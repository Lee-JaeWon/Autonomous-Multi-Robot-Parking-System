#!/usr/bin/env python3

import rospy
import cv2
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import cubic_spline_planner
class path_test:
    def __init__(self):
        rospy.init_node("path_test")
        rospy.Subscriber("/robot_1/map",OccupancyGrid,self.callback)
        rospy.Subscriber("/robot_1/move_base_simple/goal",PoseStamped,self.pathCB)
        self.path_pub = rospy.Publisher("/right_path",Path,queue_size=1)

        self.map = None
        self.mapdata = None 
        self.resolution = 0
        
        # map data
        
        self.height = 0
        self.width = 0
        
        self.frame_id = "robot_1/map"
        
        self.origin_x = 0
        self.origin_y = 0
        self.path_lst = [(0,0),(1.5,0),(1.75,1.85)]
        rospy.spin()
    def pathCB(self,data):
        rospy.loginfo("success")
        
        mx = []
        my = []
        # world coordinate to map coordinate 
        for x,y in self.path_lst:
            tx, ty = self.worldTomap(x,y)
            mx.append(float(tx))
            my.append(float(ty))
        wx = []
        wy = []

        for x,y in zip(mx,my):
            wx.append(x)
            wy.append(y)

        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(wx, wy, ds= 0.1)
        path_msg = Path()
        path_list = []
        path_msg.header.seq = rospy.Time.now()
        path_msg.header.frame_id = "robot_1/map"

        for idx,(cur_x, cur_y) in enumerate(zip(cx,cy)):
            x,y = self.mapToworld(cur_x , cur_y)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "robot_1/map"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            path_list.append(pose_msg)

        path_msg.poses = path_list

        self.path_pub.publish(path_msg)
        print("success")

    def callback(self,data):
        rospy.loginfo("map")
        
        self.height = data.info.height
        self.width = data.info.width
        self.resolution = data.info.resolution
        temp = list(data.data)
        arr = np.array([temp])
        self.map = arr.reshape(self.height,self.width)

    def mapToworld(self,mx,my):
    # OccupancyGrid의 좌표 (mx, my)를 지도상의 좌표 (wx, wy)로 변환
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return (wx,wy)
        # print(wx,wy)
        
    def worldTomap(self,wx,wy):
    # 지도상의 좌표 (wx, wy)를 OccupancyGrid의 좌표 (mx, my)로 변환
        if(wx < self.origin_x or wy < self.origin_y): return False
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        # print(mx,my)
        if (mx < self.width and my < self.height): return (mx,my)
        # return False
if __name__ =="__main__":
    start = path_test()
    