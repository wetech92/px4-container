import matplotlib.pyplot as plt
import numpy as np
import collections
import math
from .collision_check import collision_check as colli
from dataclasses import dataclass
import time
# from array import array
# import random

# @dataclass
#class Tree:
#    coord:[9999999,9999999]
#    cost:9999999
#    parent:9999999
    # def __init__(self):
    #     self.coord = np.array([-9999999,-9999999])
    #     self.cost = 9999999
    #     self.parent = 9999999

class RRT:

    def PathPlanning(self, Map, Start, Goal) :

        N_grid = len(Map)

        Start = Start.astype(np.float)
        Goal = Goal.astype(np.float)


        # User Parameter
        step_size = np.linalg.norm(Start-Goal, 2) / 500
        Search_Margin = 0

        ##.. Algorithm Initialize
        q_start = np.array([Start, 0, 0], dtype=object)       # Coord, Cost, Parent
        q_goal = np.array([Goal, 0, 0], dtype=object)

        idx_nodes = 1

        # nodes = collections.namedtuple('Tree', ['coord', 'cost', 'parent'])
        # nodes = np.array([[999999,99999], 99999, 99999])
        # nodes = np.vstack(q_start)
        nodes = q_start
        nodes = np.vstack([nodes, q_start])
        # np.vstack([q_start, q_goal])
        ##.. Algorithm Start

        flag_end = 0
        N_Iter = 0
        while (flag_end == 0):
            # Set Searghing Area
            Search_Area_min = Goal - Search_Margin
            Search_Area_max = Goal + Search_Margin
            q_rand = Search_Area_min + (Search_Area_max-Search_Area_min) * np.random.uniform(0,1,[2,1])

            # Pick the closest node from existing list to branch out from
            dist_list = []
            for i in range(0, idx_nodes+1) :
                dist = np.linalg.norm(nodes[i][0] - q_rand)
                if (i == 0) :
                    dist_list = [dist]
                else:
                    dist_list.append(dist)


            # if (idx_nodes==0) :
            #     val = dist_list
            #     idx = 0
            #     # q_near = nodes[idx]
            #     # new_coord = q_near + (q_rand - q_near) / val * step_size
            #     # q_near.coord = nodes.coord[idx]
            # else :
            val = min(dist_list)
            idx = dist_list.index(val)

            q_near = nodes[idx]
            # q_new = Tree()
            # q_new = collections.namedtuple('Tree', ['coord', 'cost', 'parent'])
            new_coord = q_near[0] + (q_rand - q_near[0]) / val * step_size




            # Collision Check
            flag_collision = colli.collision_check(Map, q_near[0], new_coord)
            #print(q_near[0], new_coord)

            # flag_collision = 0

            # Add to Tree
            if (flag_collision == 0):
                Search_Margin = 0
                new_cost = nodes[idx][1] + np.linalg.norm(new_coord - q_near[0])
                new_parent = idx
                q_new = np.array([new_coord, new_cost, new_parent], dtype=object)
                # print(nodes[0])

                nodes = np.vstack([nodes, q_new])
                # nodes = list(zip(nodes, q_new))
                # nodes.append(q_new)
                # print(nodes[0])

                Goal_Dist = np.linalg.norm(new_coord - q_goal[0])

                idx_nodes = idx_nodes + 1

                if (Goal_Dist < step_size) :
                    flag_end = 1
                    nodes = np.vstack([nodes, q_goal])
                    idx_nodes = idx_nodes + 1
            else:
                Search_Margin = Search_Margin + N_grid/100

                if Search_Margin >= N_grid :
                    Search_Margin = N_grid
            N_Iter = N_Iter + 1
            if N_Iter > 100000 :
                
                break
            # plt.imshow(Map, cmap='hot')  # , interpolation='none')
            # plt.plot(new_coord[0], new_coord[1], 'b.')
            # plt.plot(Start[0], Start[1], 'ro')
            # plt.plot(Goal[0], Goal[1], 'bo')
            # plt.show()
            # print(nodes[idx_nodes])

        flag_merge = 0
        idx = 0
        idx_parent = idx_nodes-1
        path_x_inv = np.array([])
        path_y_inv = np.array([])
        while(flag_merge == 0):
            path_x_inv = np.append(path_x_inv, nodes[idx_parent][0][0])
            path_y_inv = np.append(path_y_inv, nodes[idx_parent][0][1])
            # path_x = [path_x,nodes[idx_parent][0][0]]
            # path_y = [path_x,nodes[idx_parent][0][1]]
            # path_y[idx] = nodes[idx_parent][0][1]
            idx_parent = nodes[idx_parent][2]
            idx = idx + 1

            if idx_parent == 0 :
                flag_merge = 1

        path_x = np.array([])
        path_y = np.array([])
        for i in range(0,idx-2):
            path_x = np.append(path_x, path_x_inv[idx-i-1])
            path_y = np.append(path_y, path_y_inv[idx-i-1])




        # path_x = np.linspace(Start[0], Goal[0], 10)
        # path_y = np.linspace(Start[1], Goal[1], 10)

        return path_x, path_y
