#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""
author : LeeJaewon
email : jawwoni@naver.com
GitHub : Lee-JaeWon
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
import numpy as np
from std_msgs.msg import Bool

from matplotlib import pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.patches as patches

from math import pi, sin, cos, sqrt, atan2, asin, acos, degrees

from tf.transformations import euler_from_quaternion
import tf

class get_global_map():
    def __init__(self):

        # ----------------------------------------
        rospy.init_node('mca_in_AMRPS__get_global_map')
        rospy.loginfo_once("-- get map node --")

        # ----------------------------------------
        # Data Topic
        self.num_robots = 3
        self.map_topic = "/robot_1/map"
        self.robot_1_pose_ns = "/robot_1"
        self.robot_2_pose_ns = "/robot_2"
        self.robot_3_pose_ns = "/robot_3"
        self.PI = pi

        self.listener_one = tf.TransformListener()
        self.listener_two = tf.TransformListener()
        self.listener_thr = tf.TransformListener()


        # ----------------------------------------
        # ------Subcriber
        # map
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        # Pose
        
        # Way point
        rospy.Subscriber(self.robot_1_pose_ns + "/dp", PointStamped, self.waypointCallback, callback_args=0)
        rospy.Subscriber(self.robot_2_pose_ns + "/dp", PointStamped, self.waypointCallback, callback_args=1)
        rospy.Subscriber(self.robot_3_pose_ns + "/dp", PointStamped, self.waypointCallback, callback_args=2)
        # Goal point
        rospy.Subscriber(self.robot_1_pose_ns + "/move_base_simple/goal", PoseStamped, self.goalCallback, callback_args=0)
        rospy.Subscriber(self.robot_2_pose_ns + "/move_base_simple/goal", PoseStamped, self.goalCallback, callback_args=1)
        rospy.Subscriber(self.robot_3_pose_ns + "/move_base_simple/goal", PoseStamped, self.goalCallback, callback_args=2)
        # cmd_vel_pt from tracker.cpp
        rospy.Subscriber(self.robot_1_pose_ns + "/cmd_vel_pt", Twist, self.cmdvelptCallback, callback_args=0)
        rospy.Subscriber(self.robot_2_pose_ns + "/cmd_vel_pt", Twist, self.cmdvelptCallback, callback_args=1)
        rospy.Subscriber(self.robot_3_pose_ns + "/cmd_vel_pt", Twist, self.cmdvelptCallback, callback_args=2)
        # local_goal
        rospy.Subscriber(self.robot_1_pose_ns + "/local_goal", PoseStamped, self.localgoalCallback, callback_args=0)
        rospy.Subscriber(self.robot_2_pose_ns + "/local_goal", PoseStamped, self.localgoalCallback, callback_args=1)
        rospy.Subscriber(self.robot_3_pose_ns + "/local_goal", PoseStamped, self.localgoalCallback, callback_args=2)
        # Emergencey flag
        rospy.Subscriber("/emer_flag", Bool, self.emerCallback)
        self.emer_flag = True

        # ----------------------------------------
        # ------Publisher
        self.pub_one = rospy.Publisher(self.robot_1_pose_ns + "/cmd_vel", Twist, queue_size=10)
        self.pub_two = rospy.Publisher(self.robot_2_pose_ns + "/cmd_vel", Twist, queue_size=10)
        self.pub_thr = rospy.Publisher(self.robot_3_pose_ns + "/cmd_vel", Twist, queue_size=10)
        
        # ----------------------------------------
        # Real data
        self.mapData = None
        self.X = [[1.0, 1.0] for _ in range(self.num_robots)]
        self.RX = [[1.0, 1.0] for _ in range(self.num_robots)] # For real
        self.P = [[1.0, 1.0] for _ in range(self.num_robots)] # Pose
        self.PQ = [[1.0, 1.0, 1.0, 1.0] for _ in range(self.num_robots)] # Pose Quaternion
        self.WP = [[1.0, 1.0] for _ in range(self.num_robots)] # way point from /robot_n/dp
        self.RWP = [[1.0, 1.0] for _ in range(self.num_robots)] # For real
        self.euler = [[1.0, 1.0, 1.0] for _ in range(self.num_robots)]
        self.V_des = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.V_res = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.goal = [[1.0, 1.0] for _ in range(self.num_robots)]
        self.R_goal = [[100.0, 100.0] for _ in range(self.num_robots)] # For real
        self.vmax = [0.1, 0.6]
        self._yaw = 2 # For indexing
        self.V_pt = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.temp_V = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.LGP = [[0.0, 0.0] for _ in range(self.num_robots)]
        self.R_LGP = [[0.0, 0.0] for _ in range(self.num_robots)]

        self.robot_1_twist = Twist()
        self.robot_2_twist = Twist()
        self.robot_3_twist = Twist()

        self.trans = [[0.0, 0.0, 0.0] for _ in range(self.num_robots)]
        self.rot = [[0.0, 0.0, 0.0, 0.0] for _ in range(self.num_robots)]

        # ----------------------------------------
        self.ws_model = dict()
        self.ws_model['robot_radius'] = 40.0
        self.robot_patch = [None]*self.num_robots
        self.goal_patch = [None]*self.num_robots

    # Map
    def mapCallback(self, data):
        self.mapData = data

    def emerCallback(self, data):
        self.emer_flag = data.data

    # DP Callback for each robot
    def waypointCallback(self, data, robot_ns):
        for i in range(self.num_robots):
            if robot_ns == i:
                self.WP[i][0] = data.point.x
                self.WP[i][1] = data.point.y

    def goalCallback(self, data, robot_ns):
        for i in range(self.num_robots):
            if robot_ns == i:
                self.goal[i][0] = data.pose.position.x
                self.goal[i][1] = data.pose.position.y

    def cmdvelptCallback(self, data, robot_ns):
        for i in range(self.num_robots):
            if robot_ns == i:
                self.V_pt[i][0] = data.linear.x
                self.V_pt[i][1] = data.angular.z
                # self.vmax[i] = data.linear.x

    def localgoalCallback(self, data, robot_ns):
        for i in range(self.num_robots):
            if robot_ns == i:
                self.LGP[i][0] = data.pose.position.x
                self.LGP[i][1] = data.pose.position.y


    def get_data(self):
        figure = plt.figure()
        map_plt = figure.add_subplot(1, 1, 1)
        cmap = self.get_cmap(5)

        while not rospy.is_shutdown():
            

            map_plt.clear()

            try:
               (self.trans[0], self.rot[0]) = self.listener_one.lookupTransform('/map', '/robot_1/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            try:
               (self.trans[1], self.rot[1]) = self.listener_two.lookupTransform('/map', '/robot_2/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            try:
               (self.trans[2], self.rot[2]) = self.listener_thr.lookupTransform('/map', '/robot_3/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            if self.mapData is not None:
                

                self.map_data = self.mapData.data
                self.map_width = self.mapData.info.width
                self.map_height = self.mapData.info.height
                self.map_resolution = self.mapData.info.resolution
                self.map_startx = self.mapData.info.origin.position.x
                self.map_starty = self.mapData.info.origin.position.y

                for i in range(self.num_robots):
                    self.RX[i][0] = (self.trans[i][0] - self.map_startx)/self.map_resolution
                    self.RX[i][1] = (self.trans[i][1] - self.map_starty)/self.map_resolution
                    self.RWP[i][0] = (self.WP[i][0] - self.map_startx)/self.map_resolution
                    self.RWP[i][1] = (self.WP[i][1] - self.map_starty)/self.map_resolution

                    self.R_goal[i][0] = (self.goal[i][0] - self.map_startx)/self.map_resolution
                    self.R_goal[i][1] = (self.goal[i][1] - self.map_starty)/self.map_resolution

                    self.R_LGP[i][0] = (self.LGP[i][0] - self.map_startx)/self.map_resolution
                    self.R_LGP[i][1] = (self.LGP[i][1] - self.map_starty)/self.map_resolution

                    quaternion = (
                        self.rot[i][0],
                        self.rot[i][1],
                        self.rot[i][2],
                        self.rot[i][3]
                    )
                    self.euler[i] = euler_from_quaternion(quaternion)

                    self.P[i][0] = cos(self.euler[i][self._yaw])
                    self.P[i][1] = sin(self.euler[i][self._yaw])

                for i in range(self.num_robots):
                    self.robot_patch[i] = patches.Circle((self.RX[i][0], self.RX[i][1]),
                                                         self.ws_model['robot_radius'],
                                                         facecolor=cmap(i),
                                                         edgecolor='black',
                                                         linewidth=1.5,
                                                         ls='solid',
                                                         alpha=1,
                                                         zorder=2)
                    map_plt.add_patch(self.robot_patch[i])
                    # map_plt.arrow(self.RX[i][0], self.RX[i][1],
                    #               100*self.P[i][0],
                    #               100*self.P[i][1],
                    #               head_width=2.0, head_length=1.5, fc=cmap(i), ec=cmap(i))
                    
                    self.goal_patch[i] = patches.Rectangle((self.RWP[i][0], self.RWP[i][1]),
                                                               width=30.0, height=30.0, color=cmap(i)
                                                            )
                    map_plt.add_patch(self.goal_patch[i])
                img = np.zeros((self.map_height, self.map_width, 3), np.uint8)

                self.V_des = self.compute_V_des(X=self.RX, way=self.RWP, goal=self.R_goal) # 이후 V_desired를 Path tracker의 desired로 바꿀 가능성 (얘도 Way point를 순간 추종하는데)
                # (Path tracker와 기능이 동일하기에 우리가 구현한 속도로 가져와도 될듯)

                for i in range(self.num_robots):
                    self.temp_V[i][0] = self.P[i][0] * self.V_pt[i][0]
                    self.temp_V[i][1] = self.P[i][1] * self.V_pt[i][0]
                
                self.V_res = self.RVO_update(
                    X=self.RX, V_des=self.V_des, V_current=self.temp_V, ws_model=self.ws_model, mat_obj=map_plt)
                
                for i in range(self.num_robots):
                    map_plt.arrow(self.RX[i][0], self.RX[i][1],
                                  2250*self.V_res[i][0],
                                  2250*self.V_res[i][1],
                                  head_width=2.0, head_length=1.5, fc=cmap(i), ec=cmap(i))
                    
                    # map_plt.arrow(self.RX[i][0], self.RX[i][1],
                    #               1550*self.V_des[i][0],
                    #               1550*self.V_des[i][1],
                    #               head_width=2.0, head_length=1.5, fc=cmap(i), ec=cmap(i))

                for i in range(0, self.map_height):
                    for j in range(0, self.map_width):
                        if self.map_data[i * self.map_width + j] > 60:  # Occupied Score
                            img[i, j] = 0
                        elif self.map_data[i * self.map_width + j] == 0:  # free space
                            img[i, j] = 255
                        elif self.map_data[i * self.map_width + j] == -1:  # Unknown Space
                            img[i, j] = 100
                        else:
                            img[i, j] = 50


                if self.emer_flag is True:
                    self.robot_1_twist.linear.x = sqrt(self.V_res[0][0]**2 + self.V_res[0][1]**2)
                    self.robot_1_twist.angular.z = self.V_pt[0][1] * 1.5

                    self.robot_2_twist.linear.x = sqrt(self.V_res[1][0]**2 + self.V_res[1][1]**2)
                    self.robot_2_twist.angular.z = self.V_pt[1][1] * 1.5

                    self.robot_3_twist.linear.x = sqrt(self.V_res[2][0]**2 + self.V_res[2][1]**2)
                    self.robot_3_twist.angular.z = self.V_pt[2][1] * 1.5

                    if self.robot_1_twist.linear.x >= 0.1:
                        self.robot_1_twist.linear.x = 0.1
                    if self.robot_2_twist.linear.x >= 0.1:
                        self.robot_2_twist.linear.x = 0.1
                    if self.robot_3_twist.linear.x >= 0.1:
                        self.robot_3_twist.linear.x = 0.1

                    if self.robot_1_twist.linear.x <= -0.1:
                        self.robot_1_twist.linear.x = -0.1
                    if self.robot_2_twist.linear.x <= -0.1:
                        self.robot_2_twist.linear.x = -0.1
                    if self.robot_3_twist.linear.x <= -0.1:
                        self.robot_3_twist.linear.x = -0.1

                    if self.robot_1_twist.angular.z >= 0.6:
                        self.robot_1_twist.angular.z = 0.6
                    if self.robot_2_twist.angular.z >= 0.6:
                        self.robot_2_twist.angular.z = 0.6
                    if self.robot_3_twist.angular.z >= 0.6:
                        self.robot_3_twist.angular.z = 0.6

                    if self.robot_1_twist.angular.z <= -0.6:
                        self.robot_1_twist.angular.z = -0.6
                    if self.robot_2_twist.angular.z <= -0.6:
                        self.robot_2_twist.angular.z = -0.6
                    if self.robot_3_twist.angular.z <= -0.6:
                        self.robot_3_twist.angular.z = -0.6

                elif self.emer_flag is False:
                    self.robot_1_twist.linear.x = 0
                    self.robot_2_twist.linear.x = 0
                    self.robot_3_twist.linear.x = 0
                    self.robot_1_twist.angular.z = 0
                    self.robot_2_twist.angular.z = 0
                    self.robot_3_twist.angular.z = 0

                # print("one", self.robot_1_twist.linear.x, self.robot_1_twist.angular.z)
                print("two", self.robot_2_twist.linear.x, self.robot_2_twist.angular.z)
                print("thr", self.robot_3_twist.linear.x, self.robot_3_twist.angular.z)

                # self.pub_one.publish(self.robot_1_twist)
                self.pub_two.publish(self.robot_2_twist)
                self.pub_thr.publish(self.robot_3_twist)

                # print(self.V_res)

                map_plt.imshow(img, cmap='gray', origin='lower')
                plt.draw()
                plt.pause(1e-10)
            else:
                rospy.loginfo("self.mapData is None")

    def get_cmap(self, N):
        '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
        color_norm = colors.Normalize(vmin=0, vmax=N-1)
        scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

        def map_index_to_rgb_color(index):
            return scalar_map.to_rgba(index)
        return map_index_to_rgb_color
    
    def compute_V_des(self, X, way, goal):  # V desired
        V_des = []
        for i in range(self.num_robots):
            dif_x = [way[i][k]-X[i][k] for k in range(2)]

            norm = self.distance(dif_x, [0, 0])

            norm_dif_x = [(dif_x[k]/norm) * self.vmax[k]  for k in range(2)]  # Scaling

            # print(i, norm_dif_x)

            V_des.append(norm_dif_x[:])

            # When the robot reach to final goal, robot stop.
            if self.reach(X[i], goal[i], 2.5):
                V_des[i][0] = 0.0
                V_des[i][1] = 0.0
        return V_des
    
    def RVO_update(self, X, V_des, V_current, ws_model, mat_obj):

        ROB_RAD = ws_model['robot_radius']+0.2

        V_opt = list(V_current)

        for i in range(self.num_robots):
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            RVO_BA_all = []
            for j in range(self.num_robots):
                if i != j:

                    # Calculate with the others
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]

                    # For VO
                    # transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                    # For RVO
                    transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]

                    # For Visualization
                    # mat_obj.plot([X[i][0], X[j][0]], [
                    #              X[i][1], X[j][1]], color="green")

                    dist_BA = self.distance(pA, pB)

                    theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])

                    if 2*ROB_RAD > dist_BA:
                        dist_BA = 2*ROB_RAD

                    theta_BAort = asin(2*ROB_RAD/dist_BA)
                    theta_ort_left = theta_BA+theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA-theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    VO_BA = [transl_vB_vA, bound_left,
                             bound_right, dist_BA, 2*ROB_RAD]
                    RVO_BA_all.append(VO_BA)

            vA_post = self.intersect(pA, V_des[i], RVO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt
    
    def reach(self, p1, p2, bound=0.5):
        if self.distance(p1, p2) < bound:
            return True
        else:
            return False
        
    def distance(self, pose1, pose2):
        # compute Euclidean distance for 2D
        return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001
    
    def intersect(self, pA, vA, VO_BA_all):
        norm_v = self.distance(vA, [0, 0])
        suitable_V = []
        unsuitable_V = []
        for theta in np.arange(0, 2*self.PI, 0.1):
            for rad in np.arange(0.02, norm_v+0.02, norm_v/5.0):
                new_v = [rad*cos(theta), rad*sin(theta)]
                suit = True
                for RVO_BA in VO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_V.append(new_v)
                else:
                    unsuitable_V.append(new_v)
        new_v = vA[:]
        suit = True
        for VO_BA in VO_BA_all:
            p_0 = VO_BA[0]
            left = VO_BA[1]
            right = VO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
            if self.in_between(theta_right, theta_dif, theta_left):
                suit = False
                break
        if suit:
            suitable_V.append(new_v)
        else:
            unsuitable_V.append(new_v)
        #----------------------
        if suitable_V:
            # print 'Suitable found'
            vA_post = min(suitable_V, key=lambda v: self.distance(v, vA))
            new_v = vA_post[:]
            for VO_BA in VO_BA_all:
                p_0 = VO_BA[0]
                left = VO_BA[1]
                right = VO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
        else:
            # print 'Suitable not found'
            tc_V = dict()
            for unsuit_v in unsuitable_V:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for VO_BA in VO_BA_all:
                    p_0 = VO_BA[0]
                    left = VO_BA[1]
                    right = VO_BA[2]
                    dist = VO_BA[3]
                    rad = VO_BA[4]
                    dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        small_theta = abs(
                            theta_dif-0.5*(theta_left+theta_right))
                        if abs(dist*sin(small_theta)) >= rad:
                            rad = abs(dist*sin(small_theta))
                        big_theta = asin(abs(dist*sin(small_theta))/rad)
                        dist_tg = abs(dist*cos(small_theta)) - \
                            abs(rad*cos(big_theta))
                        if dist_tg < 0:
                            dist_tg = 0
                        tc_v = dist_tg/self.distance(dif, [0, 0])
                        tc.append(tc_v)
                tc_V[tuple(unsuit_v)] = min(tc)+0.001
            WT = 0.2
            vA_post = min(unsuitable_V, key=lambda v: (
                (WT/tc_V[tuple(v)])+self.distance(v, vA)))
        return vA_post
    
    def in_between(self, theta_right, theta_dif, theta_left):
        if abs(theta_right - theta_left) <= self.PI:
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                theta_left += 2*self.PI
                if theta_dif < 0:
                    theta_dif += 2*self.PI
                if theta_right <= theta_dif <= theta_left:
                    return True
                else:
                    return False
            if (theta_left > 0) and (theta_right < 0):
                theta_right += 2*self.PI
                if theta_dif < 0:
                    theta_dif += 2*self.PI
                if theta_left <= theta_dif <= theta_right:
                    return True
                else:
                    return False


if __name__ == '__main__':
    _map_obj = get_global_map()
    _map_obj.get_data()
