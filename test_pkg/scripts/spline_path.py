#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped
import cubic_spline_planner
import numpy as np
class Spline_path:

    def __init__ (self):

        rospy.init_node("spline_path")
        rospy.Subscriber("/robot_1/map",OccupancyGrid,self.MapCB)
        rospy.Subscriber("/robot_1/global_path",Path,self.PathCB)
        self.path_pub = rospy.Publisher("/spline_path",Path,queue_size=1)
        
                        ### map data ###
        self.height = None
        self.width = None
        self.resolution = None
        self.map = None
        self.origin_x = None
        self.origin_y = None

        rospy.spin()
    def MapCB(self,data):
        print("map subscribed")
        self.height = data.info.height
        self.width = data.info.width
        self.resolution = data.info.resolution
        
        temp = list(data.data)
        arr = np.array([temp])

        self.map = arr.reshape(data.info.height,data.info.width)
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y


    def PathCB(self,data):

        path_msg = Path()

        mx = []
        my = []
        # world coordinate to map coordinate 
        for x in data.poses:
            tx, ty = self.worldTomap(x.pose.position.x,x.pose.position.y)
            mx.append(float(tx))
            my.append(float(ty))

        wx = []
        wy = []

        for i in range(0,len(mx),15):
            wx.append(mx[i])
            wy.append(my[i])

        wx.append(mx[-1])
        wy.append(my[-1])

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

if __name__ == "__main__":
    start = Spline_path()