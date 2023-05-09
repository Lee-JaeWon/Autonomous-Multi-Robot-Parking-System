#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""
author : LeeJaewon
email : jawwoni@naver.com
GitHub : Lee-JaeWon
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np

from matplotlib import pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.patches as patches

from math import pi, sin, cos, sqrt, atan2, asin, acos, degrees

from tf.transformations import euler_from_quaternion

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

        # ----------------------------------------
        # ------Subcriber
        # map
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.mapCallback)
        # Pose
        rospy.Subscriber(self.robot_1_pose_ns + "/tracked_pose", PoseStamped, self.poseCallback, callback_args=0)
        rospy.Subscriber(self.robot_2_pose_ns + "/tracked_pose", PoseStamped, self.poseCallback, callback_args=1)
        rospy.Subscriber(self.robot_3_pose_ns + "/tracked_pose", PoseStamped, self.poseCallback, callback_args=2)
        # Way point
        rospy.Subscriber(self.robot_1_pose_ns + "/dp", PointStamped, self.waypointCallback, callback_args=0)
        rospy.Subscriber(self.robot_2_pose_ns + "/dp", PointStamped, self.waypointCallback, callback_args=1)
        rospy.Subscriber(self.robot_3_pose_ns + "/dp", PointStamped, self.waypointCallback, callback_args=2)
        
        # ----------------------------------------
        # Real data
        self.mapData = None
        self.X = [[1.0, 1.0] for i in range(self.num_robots)]
        self.P = [[1.0, 1.0] for i in range(self.num_robots)] 
        self.PQ = [[1.0, 1.0, 1.0, 1.0] for i in range(self.num_robots)] 
        self.WP = [[1.0, 1.0] for i in range(self.num_robots)] # way point from /robot_n/dp
        self.euler = [[1.0, 1.0, 1.0] for i in range(self.num_robots)] 

        # ----------------------------------------
        self.ws_model = dict()
        self.ws_model['robot_radius'] = 3.5
        self.robot_patch = [None]*self.num_robots

    # Map
    def mapCallback(self, data):
        self.mapData = data

    # Pose Callback for each robot
    def poseCallback(self, data, robot_ns):
        if self.X is not None:
            for i in range(self.num_robots):
                if robot_ns == i:
                    self.X[i][0] = data.pose.position.x
                    self.X[i][1] = data.pose.position.y
                    self.PQ[i][0] = data.pose.orientation.x
                    self.PQ[i][1] = data.pose.orientation.y
                    self.PQ[i][2] = data.pose.orientation.z
                    self.PQ[i][3] = data.pose.orientation.w

    # DP Callback for each robot
    def waypointCallback(self, data, robot_ns):
        for i in range(self.num_robots):
            if robot_ns == i:
                self.WP[i][0] = data.point.x
                self.WP[i][1] = data.point.y

    def get_data(self):
        figure = plt.figure()
        map_plt = figure.add_subplot(1, 1, 1)
        cmap = self.get_cmap(5)

        while not rospy.is_shutdown():

            map_plt.clear()

            if self.mapData is not None:

                self.map_data = self.mapData.data
                self.map_width = self.mapData.info.width
                self.map_height = self.mapData.info.height
                self.map_resolution = self.mapData.info.resolution
                self.map_startx = self.mapData.info.origin.position.x
                self.map_starty = self.mapData.info.origin.position.y

                for i in range(self.num_robots):
                    self.X[i][0] = (self.X[i][0] - self.map_startx)/self.map_resolution
                    self.X[i][1] = (self.X[i][1] - self.map_starty)/self.map_resolution
                    self.WP[i][0] = (self.WP[i][0] - self.map_startx)/self.map_resolution
                    self.WP[i][1] = (self.WP[i][1] - self.map_starty)/self.map_resolution
                    quaternion = (
                        self.PQ[i][0],
                        self.PQ[i][1],
                        self.PQ[i][2],
                        self.PQ[i][3]
                    )
                    self.euler[i] = euler_from_quaternion(quaternion)

                    self.P[i][0] = cos(self.euler[i][2])
                    self.P[i][1] = sin(self.euler[i][2])

                for i in range(self.num_robots):
                    self.robot_patch[i] = patches.Circle((self.X[i][0], self.X[i][1]),
                                                         self.ws_model['robot_radius'],
                                                         facecolor=cmap(i),
                                                         edgecolor='black',
                                                         linewidth=1.5,
                                                         ls='solid',
                                                         alpha=1,
                                                         zorder=2)
                    map_plt.add_patch(self.robot_patch[i])
                    map_plt.arrow(self.X[i][0], self.X[i][1],
                                  30*self.P[i][0],
                                  30*self.P[i][1],
                                  head_width=2.0, head_length=1.5, fc=cmap(i), ec=cmap(i))
                img = np.zeros((self.map_height, self.map_width, 3), np.uint8)

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

if __name__ == '__main__':
    _map_obj = get_global_map()
    _map_obj.get_data()
