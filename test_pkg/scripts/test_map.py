#!/usr/bin/env python3

import rospy
from MAP import MapData
import numpy as np
from nav_msgs.msg import OccupancyGrid,Path,Odometry
from geometry_msgs.msg import PoseStamped,Twist
import math
from astar import AstarPlan,State
import cubic_spline_planner
import tf,os
# /move_base_simple/goal 
# Type: geometry_msgs/PoseStamped

class TestMap():
    def __init__(self):
        
        rospy.init_node("map_node")
        rospy.Subscriber("/map",OccupancyGrid,self.callback)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.GoalCallback)
        rospy.Subscriber('/odom',Odometry,self.OdomCB)
        
        self.path_pub = rospy.Publisher("/path",Path,queue_size=1)
        self.path_pub2 = rospy.Publisher("/path2",Path,queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

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
        self.current_yaw = 0
        ## PID

        self.old_error = 0
        self.error_sum = 0

        self.old_error_h = 0
        self.error_sum_h = 0

    def OdomCB(self,data):
    
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        quaternioin = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternioin)
        self.current_yaw = euler[2]

    def pi_2_pi(self,angle): return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def GoalCallback(self,data):
        #   x: -0.5009171366691589
        #   y: 1.5592687129974365
        # print(self.current_x,self.current_y)
        ex,ey = self.worldTomap(self.current_x,self.current_y)
        # ex,ey = self.worldTomap(0.5,-1.0)
        # ex,ey = self.worldTomap(self.current_x,self.current_y)
        start_state = State(ex,ey)

        e_x = data.pose.position.x
        e_y = data.pose.position.y
        
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
        path_list1 = []
        
        path_msg.header.seq = rospy.Time.now()
        path_msg.header.frame_id = self.mapdata.getFrameId()

        ax = []
        ay = []
        for cur in plan:
            ax.append(float(cur.x))
            ay.append(float(cur.y))
            # x,y = self.mapToworld(cur.x , cur.y)
            # pose_msg = PoseStamped()
            # pose_msg.header.frame_id = self.frame_id
            # pose_msg.pose.position.x = x
            # pose_msg.pose.position.y = y
            # path_list1.append(pose_msg)
        # path_msg.poses = path_list1
        # self.path_pub2.publish(path_msg)
        # goal = [ax[-1], ay[-1]]

        bx = []
        by = []
        for i in range(0,len(ax),15):
            bx.append(ax[i])
            by.append(ay[i])
        bx.append(ax[-1])
        by.append(ay[-1])

        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(bx, by, ds= 0.1)
        path_msg = Path()
        path_list = []
        path_msg.header.seq = rospy.Time.now()
        path_msg.header.frame_id = self.mapdata.getFrameId()



        for idx,(cur_x, cur_y) in enumerate(zip(cx,cy)):
            x,y = self.mapToworld(cur_x , cur_y)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            path_list.append(pose_msg)
            # try:
            #     print(np.rad2deg(math.atan2(ay[idx+1]-ay[idx],ax[idx+1]-ax[idx])))
            # except:
            #     pass
        path_msg.poses = path_list
        self.path_pub.publish(path_msg)

        #########path traking######

        # goal = [ax[-1], ay[-1]]
        # tolerance = 0.2
        # vel_msg = Twist()
        # wgx, wgy = self.mapToworld(ax[-1],ay[-1])

        # while math.sqrt(abs(wgx - self.current_x)**2 + abs(wgy - self.current_y)**2) > tolerance :
        #     ni = self.nearest_index(zip(cx,cy))
        #     path_yaw = math.atan2(cy[ni+1]-cy[ni],cx[ni+1]-cx[ni])
        #     wpx, wpy = self.mapToworld(cx[ni],cy[ni])
        #     dxl = wpx - self.current_x
        #     dyl = wpy - self.current_y
        #     mind = math.sqrt(dxl**2 + dyl**2)
        #     angle = self.pi_2_pi(path_yaw - math.atan2(dyl, dxl))
        #     if angle < 0: mind *= -1 # angle이 0보다 작으면 마이너스 부호 붙여서 반환

        #     # right + , left -
        #     angular_error = path_yaw - self.current_yaw
        #     angular = self.PID_controller_h(angular_error)

        #     # cte = self.PID_controller(mind)
        #     vel_msg.linear.x = 0.2
        #     vel_msg.angular.z = angular + mind * 0.25
        #     self.vel_pub.publish(vel_msg)    
        #     rospy.sleep(0.033)

        # vel_msg.linear.x = 0 
        # vel_msg.angular.z = 0
        # self.vel_pub.publish(vel_msg)



        # goal = [ax[-1], ay[-1]]
        # # # cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
        # target_speed = 0.2  # simulation parameter km/h -> m/s
        # sp = calc_speed_profile(cyaw, target_speed)
        # t, x, y, yaw, v = do_simulation(cx, cy, cyaw, ck, sp, goal)
        # vel_msg = Twist()

    def PID_controller_h(self,error):
        P = 1.75
        I = 0.03
        D = 0.5
        self.error_sum_h += error
        derivative_term = (error - self.old_error_h) / 0.033
        proportion = error
        self.old_error_h = error
        return (proportion * P + self.error_sum_h * I + derivative_term * D)

    def PID_controller(self,error):

        P = 1.0
        I = 0.02
        D = 0.1
        self.error_sum += error
        derivative_term = (error - self.old_error) / 0.033
        proportion = error
        self.old_error = error
        return (proportion * P + self.error_sum * I + derivative_term * D) * -1

    def nearest_index(self,path):
        min_dst = int(1e8)
        min_idx = int(1e8)

        for idx ,(mx,my) in enumerate(path):
            x,y = self.mapToworld(mx,my)
            cur_dst = math.sqrt(abs(x-self.current_x)**2 + abs(y-self.current_y))
            if cur_dst< min_dst:
                min_dst = cur_dst
                min_idx = idx
        return min_idx

    def callback(self,data):

        self.height = data.info.height
        self.width = data.info.width
        self.resolution = data.info.resolution
        
        temp = list(data.data)
        arr = np.array([temp])
        self.map = arr.reshape(data.info.height,data.info.width)

        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y

        self.mapdata = MapData(self.map,self.resolution,self.height,self.width,self.origin_x,self.origin_y)
        
        rospy.loginfo("map size : {} x {} ".format(self.mapdata.getCellSizeX(),self.mapdata.getCellSizeY()))
        rospy.loginfo("map resolution : {}".format(self.mapdata.getResolution() ) )
        rospy.loginfo("map origin_x : {} origin_y : {}".format(self.mapdata.getOriginX(),self.mapdata.getOriginY()))
        # print(self.worldTomap(0,0))
        
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
