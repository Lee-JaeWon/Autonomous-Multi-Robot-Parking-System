#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import pickle
import numpy as np
from itertools import product
from math import cos, sin, pi, sqrt 
import collections

from priority_queue import priority_dict

class State(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, state):
        return state and self.x == state.x and self.y == state.y 

    def __hash__(self):
        return hash((self.x, self.y))
    
    
class AStarPlanner(object):
    """
    Applies the A* shortest path algorithm on a given grid world
    """
    
    def __init__(self, world):
        # (rows, cols, channels) array with values in {0,..., 255}
        self.world = world.sum(axis=2) / 3
        # print('shape : ',self.world.shape)
        # print('len : ',len(self.world))

        # (rows, cols) binary array. Cell is 1 iff it is occupied
        self.occ_grid = self.world
        self.occ_grid = (self.occ_grid  < 255).astype('uint8')
        # print('grid map  size :  600 x 1000')
    
        # print('max : ',self.occ_grid.max())
        # print((self.occ_grid[state.x-1:state.x+1, state.y-1:state.y+1] == 0).all())
        
    def state_is_free(self, state):
        """
        Does collision detection. Returns true iff the state and its nearby 
        surroundings are free.
        """
        # check = (self.occ_grid[state.x-3:state.x+3, state.y-3:state.y+3] == 1).all()
        # print(check)
        return (self.occ_grid[state.x-1:state.x+1, state.y-1:state.y+1] == 0).all()
        
    def get_neighboring_states(self, state):
        """
        Returns free neighboring states of the given state. Returns up to 8
        neighbors (north, south, east, west, northwest, northeast, southwest, southeast)
        """
        
        x = state.x
        y = state.y
        # print(x , y )
        rows, cols = self.world.shape[:2]
        # print('self.world.shape[:2] : ',self.world.shape[:2])
        # row  600
        # cols 1000

        dx = [0]
        dy = [0]
        
        if (x > 0):
            dx.append(-1)

        if (x < rows -1):
            dx.append(1)

        if (y > 0):
            dy.append(-1)

        if (y < cols -1):
            dy.append(1)
            

        '''iterator1 = [1, 2, 3]
            iterator2 = ['A', 'B', 'C']

            product(iterator1, iterator2)))

             [(1, 'A'), (1, 'B'), (1, 'C'), (2, 'A'), (2, 'B'), (2, 'C'), (3, 'A'), (3, 'B'), (3, 'C')]     '''
        # product() returns the cartesian product
        # yield is a python generator. Look it up.
        for delta_x, delta_y in product(dx,dy):

            if delta_x != 0 or delta_y != 0:
                ns = State(x + delta_x, y + delta_y)
                # print("ns : ", ns.x , ns.y)
                if self.state_is_free(ns): # 상태가 프리면 하나씩 보낸다.
                    yield ns 
            

    def _follow_parent_pointers(self, parents, state):
        """
        Assumes parents is a dictionary. parents[key]=value
        means that value is the parent of key. If value is None
        then key is the starting state. Returns the shortest
        path [start_state, ..., destination_state] by following the
        parent pointers.
        """
        
        assert (state in parents)
        curr_ptr = state
        shortest_path = [state]
        
        while curr_ptr is not None:
            shortest_path.append(curr_ptr)
            curr_ptr = parents[curr_ptr]

        # return a reverse copy of the path (so that first state is starting state)
        return shortest_path[::-1]

    #
    # TODO: this method currently has the implementation of Dijkstra's algorithm.
    # Modify it to implement A*. The changes should be minor.
    #
    def plan(self, start_state, dest_state):
        """
        Returns the shortest path as a sequence of states [start_state, ..., dest_state]
        if dest_state is reachable from start_state. Otherwise returns [start_state].
        Assume both source and destination are in free space.
        
        """
        print('start_state : ',(start_state.x,start_state.y) , 'dest_state : ',(dest_state.x,dest_state.y))
        assert (self.state_is_free(start_state)) # 시작과 끝은 프리 상태여야 한다. 아니면 assert 에러가 뜬다.
        assert (self.state_is_free(dest_state))

        # Q is a mutable priority queue implemented as a dictionary
        Q = priority_dict()
        Q[start_state] = 0.0
        # Array that contains the optimal distance to come from the starting state
        dist_to_come = float("inf") * np.ones((world.shape[0], world.shape[1])) # 무한대로 초기화
        # print(dist_to_come)
        dist_to_come[start_state.x,start_state.y] = 0 # 시작지점 0으로 초기화 
        # Boolean array that is true iff the distance to come of a state has been
        # finalized
        evaluated = np.zeros((world.shape[0], world.shape[1]), dtype='uint8')
        # print('evaluated : ',evaluated)
        # Contains key-value pairs of states where key is the parent of the value
        # in the computation of the shortest path
        parents = {start_state: None}
        while Q:

            # print('len(Q) : ',len(Q))
            # s is also removed from the priority Q with this
            s = Q.pop_smallest()
            # print("s : ",s.x , s.y)
            # print('distance : ',dist_to_come[s.x,s.y])
            # Assert s hasn't been evaluated before
            assert (evaluated[s.x, s.y] == 0) 
            print("s : ",s.x,s.y)

            evaluated[s.x, s.y] = 1 # 들렸으면 1로 바꾸기
            if s == dest_state: # s가 목적지면 포인트 찍기
                print("finish")
                return self._follow_parent_pointers(parents, s)
            # for all free neighboring states
            for ns in self.get_neighboring_states(s):
                if evaluated[ns.x, ns.y] == 1: # 넥스트 스테이트가 이미 들린 곳이라면 continue
                    continue
                shortest_dst = self.heuristic(s,ns)# 지금 위치에서 갈 수 있는 곳 까지의 비용
                calculated_dst = dist_to_come[s.x, s.y]  + shortest_dst # 지금 위치 더하기 지금 위치에서 넥스트 스테이트 까지 비용
                # print('before : ',calculated_dst,dist_to_come[ns.x, ns.y])
                if  (ns not in Q) or (calculated_dst < (dist_to_come[ns.x, ns.y])):# 이미 계산된 곳 피하기 위해서 최단거리가 아닌 곳은 evaluated != 1 
                    # print(calculated_dst,dist_to_come[ns.x, ns.y])
                    # time.sleep(0.1)
                    Q[ns] = dist_to_come[s.x, s.y] + self.heuristic(ns,dest_state)# 넥스트 스테이트에서 목표까지 거리 계산 한 값 큐에 넣어주고 가장 작은 값 반환
                    dist_to_come[ns.x, ns.y] = dist_to_come[s.x, s.y] + self.heuristic(s,ns)# 누적거리 계산
                    parents[ns] = s # ns가 어디서부터 왔는지 
            # print(dist_to_come[s.x -3 : s.x + 3 , s.y - 3 : s.y + 3])
                # print('qns',Q[ns])
        return [start_state]

    def heuristic(self,current,next):
        x1, y1 = next.x , next.y
        x2, y2 = current.x , current.y
        dx = x1-x2
        dy = y1-y2
        return sqrt(dx**2 + dy**2)
        # return dx + dy
    
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print ("Usage: astar_planner.py occupancy_grid.pkl")
        
        sys.exit(1)

    # pkl_file = open(sys.argv[1], 'rb')

    # # world is a numpy array with dimensions (rows, cols, 3 color channels)
    # world = pickle.load(pkl_file)
    # pkl_file.close()
    world = np.load(sys.argv[1])
    # k_world = np.zeros((world.shape[0], world.shape[1],3))
    # k_world[:,:,0], k_world[:,:,1], k_world[:,:,2] = world, world, world
    
    
    print(world.shape)
    astar = AStarPlanner(world)
    # print(world.max())

    start_state = State(275, 790)
    dest_state = State(115, 900)
    
    plan = astar.plan(start_state, dest_state)
    print(len(plan))
    