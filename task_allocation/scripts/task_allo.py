#!/usr/bin/env python3
#-*- coding: utf-8 -*-

"""
author : LeeJaewon
email : jawwoni@naver.com
GitHub : Lee-JaeWon
"""

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance_matrix

from matplotlib import pyplot as plt
import numpy as np
import tf


class task_allocation():
    def __init__(self):
        rospy.init_node("Task_Allocation")
        rospy.loginfo_once("Task_Allocation Node")

        self.rate = rospy.Rate(30) # 30hz

        self.n_robots = 3
        self.n_task = 6

        # self.robot_pos = [[0.0, 0.0] for _ in range(self.n_robots)]
        self.robot_pos = np.zeros(shape=(self.n_robots, 2))
        self.task_pos = np.zeros(shape=(self.n_task, 2))
        self.robot_act_flag = [False for _ in range(self.n_robots)]

        # Subscriber
        self.robot_1_pose_ns = "/robot_1"
        self.robot_2_pose_ns = "/robot_2"
        self.robot_3_pose_ns = "/robot_3"

        rospy.Subscriber("/task_num", Int8, self.tasknumCallback) # from hyedo

        # Robot 활동중인지에 관한 Flag를 Subscribe
        rospy.Subscriber(self.robot_1_pose_ns + "/robot_act", Bool, self.robotactCallback, callback_args=0) # from hyedo
        rospy.Subscriber(self.robot_2_pose_ns + "/robot_act", Bool, self.robotactCallback, callback_args=1) # from hyedo
        rospy.Subscriber(self.robot_3_pose_ns + "/robot_act", Bool, self.robotactCallback, callback_args=2) # from hyedo

        # Pose
        self.trans = [[0.0, 0.0, 0.0] for _ in range(self.n_robots)]
        self.rot = [[0.0, 0.0, 0.0, 0.0] for _ in range(self.n_robots)]
        self.listener = tf.TransformListener()

        # Create figure
        self.fig, self.ax = plt.subplots(1,3,figsize=(10,5))
        plt.rc('font', size=16) 

        # Initialize data
        self.Update_robot_init_pose()
        self.Update_task_init_pose()

    def tasknumCallback(self, data):
        self.n_task = data.data

    def robotactCallback(self, data, robot_ns):

        for i in range(self.n_robots):
            if robot_ns == i:
                self.robot_act_flag[i] = data.data

    def run(self):
        while not rospy.is_shutdown():
            # rospy.loginfo(self.robot_pos)

            self.Update_tf()

            self.Clear_vis()

            # self.Update_robot_pose()

            self.Update_vis()
            self.task_allocation()
            self.task_allocation_greedy()

            print(self.row_ind, self.col_ind)

            for i in range(3):
                self.robot_pos[i][0] = self.robot_pos[i][0] + 0.01
                self.robot_pos[i][1] = self.robot_pos[i][1] + 0.01
            
            plt.pause(1e-10)

            self.rate.sleep()

    # Initialize
    def Update_robot_init_pose(self):
        """
        Robot의 초기 위치 설정 함수.
        """
        x = 0; y = 1
        self.robot_pos[0][x] = -0.73
        self.robot_pos[0][y] =  0.13
        self.robot_pos[1][x] = -0.73
        self.robot_pos[1][y] = -0.54
        self.robot_pos[2][x] = -0.73
        self.robot_pos[2][y] = -1.21

    # Initialize
    def Update_task_init_pose(self):
        """
        Task의 초기 위치 설정 함수.
        """
        x = 0; y = 1
        self.task_pos[0][x] =  0.54
        self.task_pos[0][y] =  0.0
        self.task_pos[1][x] =  0.54
        self.task_pos[1][y] = -0.56
        self.task_pos[2][x] =  0.54
        self.task_pos[2][y] = -1.21

        self.task_pos[3][x] =  0.0
        self.task_pos[3][y] =  1.5
        self.task_pos[4][x] =  1.0
        self.task_pos[4][y] =  1.5
        self.task_pos[5][x] =  2.0
        self.task_pos[5][y] =  1.5

    def Update_robot_pose(self):
        """
        Robot의 실제 위치 업데이트 함수.(From TF transfrom)
        """
        x = 0; y = 1
        for i in range(self.n_robots):
            self.robot_pos[i][0] = self.trans[i][0]
            self.robot_pos[i][1] = self.trans[i][1]

    def Update_task_pose(self):
        """
        Task의 실제 위치 업데이트 함수.(From Task Callback)
        """
        x = 0; y = 1
        self.task_pos = np.zeros(shape=(self.n_task, 2))
        for i in range(self.n_task):
            self.task_pos[i][0] = i
            self.task_pos[i][1] = i

    def Update_tf(self):
        """
        Robot의 실제 위치 변환.(from /map and /base_link)
        """
        for i in range(self.n_robots):
            self.robot_ns = "/" + "robot_" + str(i+1) + "/base_link"
            try:
                (self.trans[0], self.rot[0]) = self.listener.lookupTransform('/map', self.robot_ns, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def Clear_vis(self):
        """
        Visualization 초기화
        """
        self.ax[0].cla()
        self.ax[1].cla()
        self.ax[2].cla()

    def Update_vis(self):
        """
        Visualization 업데이트
        """
        self.ax[0].plot(self.robot_pos[:,0], self.robot_pos[:,1], 'k^',label="Robot")
        self.ax[0].plot(self.task_pos[:,0], self.task_pos[:,1], 'bo',label="Task")
        self.ax[0].set(xlim=(-3.0,3.0),ylim=(-3.0,3.0))
        self.ax[0].set_aspect("equal")
        self.ax[0].legend()
        self.ax[0].set_title("Environment")

        self.ax[1].plot(self.robot_pos[:,0], self.robot_pos[:,1], 'k^',label="Robot")
        self.ax[1].plot(self.task_pos[:,0], self.task_pos[:,1], 'bo',label="Task")
        self.ax[1].set(xlim=(-3.0,3.0),ylim=(-3.0,3.0))
        self.ax[1].set_aspect("equal")
        self.ax[1].legend()

        self.ax[2].plot(self.robot_pos[:,0], self.robot_pos[:,1], 'k^',label="Robot")
        self.ax[2].plot(self.task_pos[:,0], self.task_pos[:,1], 'bo',label="Task")
        self.ax[2].set(xlim=(-3.0,3.0),ylim=(-3.0,3.0))
        self.ax[2].set_aspect("equal")
        self.ax[2].legend()
        
        
    def task_allocation(self):
        """
        Hungarian Algorithm 수행
        """
        self.cost = distance_matrix(self.robot_pos, self.task_pos)
        self.row_ind, self.col_ind = linear_sum_assignment(self.cost)
        self.hungarian_total_cost = self.cost[self.row_ind, self.col_ind].sum()
        self.ax[1].set_title("Hungarian algorithm cost: {:.3}".format(self.hungarian_total_cost))
        for i in range(len(self.col_ind)):
            self.ax[1].plot([self.robot_pos[self.row_ind,0],self.task_pos[self.col_ind,0]],[self.robot_pos[self.row_ind,1],self.task_pos[self.col_ind,1]],'r--')

    def task_allocation_greedy(self):
        """
        Greedy Algorithm 수행
        """
        greedy_task = []
        self.greedy_total_cost = 0
        for robot in range(self.n_robots):
            cand_task = list(np.argsort(self.cost[robot, :]))
            while True:
                task = cand_task[0]
                if task not in greedy_task:
                    greedy_task.append(task)
                    self.greedy_total_cost += self.cost[robot, task]
                    break
                else:
                    cand_task.pop(0)

        greedy_task = np.array(greedy_task)

        for i in range(len(self.col_ind)):
            self.ax[2].plot([self.robot_pos[self.row_ind,0],self.task_pos[greedy_task,0]],[self.robot_pos[self.row_ind,1],self.task_pos[greedy_task,1]],'r--')
        self.ax[2].set_title("Greedy algorithm cost: {:.3}".format(self.greedy_total_cost))

        if self.hungarian_total_cost <= self.greedy_total_cost :
            self.ax[0].set_title("Hungarian algorithm is better.")
            self.fig.set_facecolor("#40CBEA")
        else:
            self.ax[0].set_title("Greedy algorithm is better.")
            self.fig.set_facecolor("#FFCAF8")

if __name__ == '__main__':
    task_obj = task_allocation()
    task_obj.run()