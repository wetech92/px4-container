import time
import matplotlib.pyplot as plt
import numpy as np
import time

import onnx
import onnxruntime as ort
import copy

import cv2


import collections
import math
from .collision_check import collision_check as colli
from dataclasses import dataclass


#   ROS2 python 
import rclpy
from rclpy.node import Node
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathPlanningSetpoint


# Opencv-ROS
import cv2


class DeepSACNode(Node):
    def __init__(self):
        super().__init__('SAC_module')
        ##  Input
        self.response_timestamp = 0
        self.requestFlag = False
        self.requestTimestamp = 0
        self.start_point = []
        self.goal_point = []
        ##  Output
        self.response_timestamp = 0
        self.response_pathplanning = False
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_z = []
        self.waypoint_index = 0
        self.waypoint_lenth = 0
        self. qosProfileGen()
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.SACmoduleService_ = self.create_service(PathPlanningSetpoint, 'path_planning', self.PathPlanningCallback)
        print("=== Path Planning Node is Running =====")
        
    def PathPlanningCallback(self, request, response):
        print("===== Request PathPlanning Node =====")
        '''
        uint64 request_timestamp	# time since system start (microseconds)
        bool request_pathplanning
        float64[] start_point
        float64[] goal_point
        '''
        self.requestFlag = request.request_pathplanning
        self.requestTimestamp = request.request_timestamp
        self.start_point = request.start_point
        self.goal_point = request.goal_point
        self.Image = cv2.imread("/root/ros_ws/src/a4vai/a4vai/path_planning/Map/RawImage.png")
        if self.requestFlag is True : 
            # self.waypoint_x, self.waypoint_y, self.waypoint_lenth = SAC.PathPlanning(self, self.Image, self.start_point, self.goal_point)
            print("===== PathPlanning Complete!! =====")
            '''
            uint64 response_timestamp	# time since system start (microseconds)
            bool response_pathplanning
            float64[] waypoint_x
            float64[] waypoint_y
            float64[] waypoint_z
            uint32 waypoint_index
            '''
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = True
            response.waypoint_x = self.waypoint_x
            response.waypoint_y = self.waypoint_y
            response.waypoint_z = self.waypoint_z
            response.waypoint_index = self.waypoint_index
            print("===== Response PathPlanning Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = False
            response.waypoint_x = [0] * 5
            response.waypoint_y = [0] * 5
            response.waypoint_z = self.waypoint_z
            response.waypoint_index = self.waypoint_index
            print("===== Can't Response PathPlanning Node =====")
            return response
            
    def qosProfileGen(self):
        #   Reliability : 데이터 전송에 있어 속도를 우선시 하는지 신뢰성을 우선시 하는지를 결정하는 QoS 옵션
        #   History : 데이터를 몇 개나 보관할지를 결정하는 QoS 옵션
        #   Durability : 데이터를 수신하는 서브스크라이버가 생성되기 전의 데이터를 사용할지 폐기할지에 대한 QoS 옵션
        self.QOS_Sub_Sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        self.QOS_Service = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        
    def TimesyncCallback(self, msg):
        self.response_timestamp = msg.timestamp

class SAC:

    def PathPlanning(self, Map, Start, Goal) :
        onnx_model = onnx.load('/root/ros_ws/src/integration/integration/PathPlanning/RRT/test26.onnx')
        onnx.checker.check_model(onnx_model)
        ort_session = ort.InferenceSession('/root/ros_ws/src/integration/integration/PathPlanning/RRT/test26.onnx')

        MapSize = 5000
        Step_Num = 5000

        # Airsim Image
        RawImage = (cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png", cv2.IMREAD_GRAYSCALE))
        # Gazebo Image
        RawImage = (cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/test.png", cv2.IMREAD_GRAYSCALE))
        Image = np.uint8(np.uint8((255 - RawImage)/ 255))
        Image_New = np.zeros((MapSize, MapSize))

        Start1 = time.time()

        Init = np.array([Start[0], 2, Start[1]], dtype=float)
        Target = np.array([Goal[0], 2, Goal[1]], dtype=float)

        Waypoint = np.zeros((Step_Num, 3))
        Pos = Init
        ds = 0
        Obs_mat = np.array([[]])
        for i in range(1, Step_Num):
            ## Initialize
            Obs1 = np.zeros((1, 3))
            Obs2 = np.zeros((1, 3))
            Obs3 = np.zeros((1, 12))

            ## Make Obs3(Lidar Terms)
            MaxLidar = 250
            for j in range(0, 12):  # 0 - 11
                LidarState = Pos + 1*ds
                for k in range(1, 5):
                    LidarState = LidarState + 50.0 * np.array([np.cos( ((30)*(j-1)) * np.pi / 180), 0, np.sin( ((30)*(j-1)) * np.pi / 180)])  # Radian으로 해야 함

                    # Map Size 넘어갈만큼 진전시켰으면 Obs3 0으로 하고 Data 수집 멈추기
                    if LidarState[0] >= MapSize or LidarState[2] >= MapSize:
                        Diff = LidarState - (Pos + 1*ds)
                        if np.linalg.norm(Diff) <= MaxLidar:
                            Obs3[0][j] = (np.linalg.norm(Diff)) / 50    # Scaling 필요
                            break
                        else:
                            Obs3[0][j] = MaxLidar / 50
                            break

                    if LidarState[0] < 0 or LidarState[2] < 0:
                        Diff = LidarState - (Pos + 1*ds)
                        if np.linalg.norm(Diff) <= MaxLidar:
                            Obs3[0][j] = (np.linalg.norm(Diff)) / 50   #s Scaling 필요
                            break
                        else:
                            Obs3[0][j] = MaxLidar / 50
                            break

                    if Image_New[int(LidarState[2])][int(LidarState[0])] > 0:  # 꼭 후처리된 Map에서 # 안의 순서 주의
                        Diff = LidarState - (Pos + 1*ds)
                        if np.linalg.norm(Diff) <= MaxLidar:
                            Obs3[0][j] = (np.linalg.norm(Diff)) / 50    # Scaling 필요
                            break
                        else:
                            Obs3[0][j] = MaxLidar / 50
                            break


                    Obs3[0][j] = MaxLidar / 50

            # Observation for onnx range [-2500, 2500]
            Obs1 = (Target - Init) / np.linalg.norm(Target - Init)
            Obs2 = (Target - Pos) / 50

            ## Make Observation
            Obs = np.random.randn(1, 18)
            Obs[0][2] = Obs1[2]
            Obs[0][3] = Obs2[0]
            Obs[0][4] = Obs2[1]
            Obs[0][5] = Obs2[2]

            Obs[0][6] = Obs3[0][0]
            Obs[0][7] = Obs3[0][1]
            Obs[0][8] = Obs3[0][2]
            Obs[0][9] = Obs3[0][3]
            Obs[0][10] = Obs3[0][4]
            Obs[0][11] = Obs3[0][5]
            Obs[0][12] = Obs3[0][6]
            Obs[0][13] = Obs3[0][7]
            Obs[0][14] = Obs3[0][8]
            Obs[0][15] = Obs3[0][9]
            Obs[0][16] = Obs3[0][10]
            Obs[0][17] = Obs3[0][11]

            Act = ort_session.run(None, {"obs_0": Obs.astype(np.float32)})

            ## Make Move
            Act = Act[2][0][0]  # np.linalg.norm(Act[2][0] - Act[2][1])

            for j in range(0, 5):  # 0, 1, 2, 3, 4
                LOS_2D_Norm = (Target - Pos) / np.linalg.norm(Target - Pos)
                Action_Vec = np.array([LOS_2D_Norm[2], 0, -LOS_2D_Norm[0]])
                Vel = 3.0 * LOS_2D_Norm  # Original 5
                Action_Vec = 3.0 * Act * 2 * Action_Vec
                ds = 1 * Vel + Action_Vec
                Pos = Pos + ds

            ## Set End Condtion
            if (np.linalg.norm(Target - Pos) < 25):
                break

            Waypoint[i][0] = Pos[0]
            Waypoint[i][1] = Pos[1]
            Waypoint[i][2] = Pos[2]

        path_x = np.array([])
        path_y = np.array([])

        # path_x = np.append(path_x,Init[0])
        # path_y = np.append(path_y,Init[2])
        for l in range(1,i):  # 0 - 4999
            path_x = np.append(path_x, Waypoint[l][0])
            path_y = np.append(path_y, Waypoint[l][2])
        path_x = np.append(path_x, Target[0])
        path_y = np.append(path_y, Target[2])

        ## Plot and Save Image
        imageLine = RawImage.copy()

        # 이미지에 맞게 SAC Waypoint 변경 후 그리기
        for m in range(0, i-2):
            Im_i = int(path_x[m+1])
            Im_j = MapSize - int(path_y[m+1])

            Im_iN = int(path_x[m+2])
            Im_jN = MapSize - int(path_y[m+2])

            cv2.line(imageLine, (Im_i, Im_j), (Im_iN, Im_jN), (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        cv2.imwrite('TestResult0.png', imageLine)  ################################


        ## For Pruning
        # Waypoint
        wp_epi = []
        for i in range(0,len(path_x)):
            wp_epi.append([path_x[i], path_y[i]])

        origin_wp = copy.copy(wp_epi)
        wp_candidate = origin_wp[len(origin_wp) - 1]  # 마지막 WP
        pruned_wp = [wp_candidate]  # 마지막 WP

        wp_index_pre = origin_wp.index(wp_candidate)  # 마지막 WP의 인덱스

        while 1:
            for i in range(0, len(origin_wp)):
                collisionCheck = False

                InitPos = np.array([wp_candidate[0], wp_candidate[1]])  # wp_candidate 초기 위치
                FinalPos = np.array([origin_wp[i][0], origin_wp[i][1]])

                Unit = (FinalPos - InitPos) / np.linalg.norm((FinalPos - InitPos))

                CurrPos = InitPos
                for j in range(0, 200):
                    CurrPos = CurrPos + 1.0 * Unit

                    if CurrPos[0] > MapSize or CurrPos[1] > MapSize:
                        collisionCheck = True
                        break

                    if CurrPos[0] < 0 or CurrPos[1] < 0:
                        collisionCheck = True
                        break

                    if Image_New[int(CurrPos[1])][int(CurrPos[0])] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break

                    if wp_candidate == origin_wp[len(origin_wp) - 1] and InitPos[0] == origin_wp[0][0]:  # 마지막 WP
                        collisionCheck = True
                        break

                    if np.linalg.norm(CurrPos - FinalPos) < 10:  # 장애물 없이 다음 WP까지 가면 반복문 나오기(collisionCheck 0으로 해서)
                        break

                if collisionCheck == False:
                    wp_candidate = origin_wp[i]  # 중간 경로점 없앰
                    wp_index = origin_wp.index(wp_candidate)
                    # pruned_wp.append(wp_candidate)
                    # wp_index_pre = wp_index
                    if wp_index_pre - wp_index < 0.2 * (len(origin_wp) - 1): #(0.01 * pruning_rate) * (len(origin_wp) - 1):  # 지금 인덱스와 크기 차이가 28보다 작으면 첫번째 반복문 끝내기 # 차이 28만큼 진행시켰으면 200
                        pruned_wp.append(wp_candidate)
                        wp_index_pre = wp_index
                        break

            if wp_candidate == origin_wp[0]:  # 진짜 끝
                break
            elif len(pruned_wp) == 1 and wp_candidate == pruned_wp[0]:
                break
            elif len(pruned_wp) > 1 and wp_candidate == pruned_wp[-2]:
                break

        pruned_wp = list(reversed(pruned_wp))

        pruned_x_points = []
        pruned_y_points = []
        for i in range(len(pruned_wp)):
            pruned_x_points.append(pruned_wp[i][0])
            pruned_y_points.append(pruned_wp[i][1])

        ## Plot and Save Image
        imageLine = RawImage.copy()

        # 이미지에 맞게 pruned up Waypoint 변경 후 그리기
        for m in range(0, i-2):
            Im_i = int(pruned_x_points[m+1])
            Im_j = MapSize - int(pruned_y_points[m+1])

            Im_iN = int(pruned_x_points[m+2])
            Im_jN = MapSize - int(pruned_y_points[m+2])

            cv2.line(imageLine, (Im_i, Im_j), (Im_iN, Im_jN), (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        cv2.imwrite('TestResult_prunning0.png' ,imageLine)  ################################

        return pruned_x_points, pruned_y_points, len(pruned_x_points)

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

        flag_merge = 0
        idx = 0
        idx_parent = idx_nodes-1
        path_x_inv = np.array([])
        path_y_inv = np.array([])
        while(flag_merge == 0):
            path_x_inv = np.append(path_x_inv, nodes[idx_parent][0][0])
            path_y_inv = np.append(path_y_inv, nodes[idx_parent][0][1])

            idx_parent = nodes[idx_parent][2]
            idx = idx + 1

            if idx_parent == 0 :
                flag_merge = 1

        path_x = np.array([])
        path_y = np.array([])
        for i in range(0,idx-2):
            path_x = np.append(path_x, path_x_inv[idx-i-1])
            path_y = np.append(path_y, path_y_inv[idx-i-1])


        return path_x, path_y
