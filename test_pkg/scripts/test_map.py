#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid,Path,Odometry

from geometry_msgs.msg import PointStamped,Pose,PoseStamped
from itertools import product
import time
from math import cos, sin, pi, sqrt 
from MAP import MapData
from astar import AstarPlan,State


# /move_base_simple/goal 
# Type: geometry_msgs/PoseStamped

class TestMap():
    def __init__(self):
        
        rospy.init_node("map_node")
        rospy.Subscriber("map",OccupancyGrid,self.callback)
        rospy.Subscriber('/clicked_point',PointStamped,self.PointCB)
        rospy.Subscriber('odom',Odometry,self.OdomCB)
        self.path_pub = rospy.Publisher("/path",Path,queue_size=1)
        self.map = None
        self.mapdata = None 
        self.resolution = 0
        # map data
        self.height = 0
        self.width = 0
        
        self.frame_id = "map"
        
        self.origin_x = 0
        self.origin_y = 0
        
        self.current_x = 0
        self.current_y = 0
        
    def OdomCB(self,data):
    
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        
    def PointCB(self,data):
        
        #   x: -0.5009171366691589
        #   y: 1.5592687129974365
        # ex,ey = self.worldTomap(-0.500,1.55)
        ex,ey = self.worldTomap(self.current_x,self.current_y)
        
        start_state = State(ex,ey)
        e_x = data.point.x
        e_y = data.point.y
        
        if self.worldTomap(e_x,e_y) == False:
            rospy.loginfo('unreachable area!')
            return
        else: x,y = self.worldTomap(e_x,e_y)
        
        if self.mapdata.getMap()[x,y] != 0: 
            rospy.loginfo("unreachable area!")
            return
        
        rospy.loginfo("start_points : {} , {}".format(ex,ey)) 
        rospy.loginfo("goal_points : {} , {}".format(x,y))
        
        dest_state = State(x,y)
        astar = AstarPlan(self.mapdata.getMap(),self.mapdata.getCellSizeX(),self.mapdata.getCellSizeY())
        plan = astar.plan(start_state, dest_state)
        rospy.loginfo("Path Planning Successed")
        # paths in plan
        
        path_msg = Path()
        path_list = []
        
        path_msg.header.seq = rospy.Time.now()
        path_msg.header.frame_id = self.mapdata.getFrameId()
        
        for cur in plan:
            x,y = self.mapToworld(cur.x , cur.y)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            path_list.append(pose_msg)
            
        path_msg.poses = path_list
        self.path_pub.publish(path_msg)
        
    def callback(self,data):
        
        self.height = data.info.height
        self.width = data.info.width
        self.resolution = data.info.resolution
        
        temp = list(data.data)
        arr = np.array([temp])
        self.map = arr.reshape(self.height,self.width)
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        print(self.origin_x, self.origin_y)
        rospy.loginfo("map size : {} x {} ".format(self.height,self.width))
        rospy.loginfo("map resolution : {}".format(self.resolution))
        rospy.loginfo("map origin_x : {} origin_y : {}".format(self.origin_x,self.origin_y))

        self.mapdata = MapData(self.map,self.resolution,self.height,self.width,self.origin_x,self.origin_y)
        
        self.worldTomap(self.origin_x,self.origin_y)
        self.mapToworld(0,0)
        print(self.worldTomap(0,0))
        
        # for i in self.map:
        #     print(i)
        #     time.sleep(0.1)
            
        # astar = AstarPlan(self.mapdata.getMap())
        # start_state = State(0, 0)
        # dest_state = State(90, 90)
        # plan = astar.plan(start_state, dest_state)
        # for cur in plan:
        #     print(self.mapToworld(cur.x , cur.y))
        
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

if (__name__ == "__main__"):
    start = TestMap()
    rospy.spin()
