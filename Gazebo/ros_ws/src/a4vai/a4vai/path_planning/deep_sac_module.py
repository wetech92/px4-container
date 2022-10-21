import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import math

# from .collision_check import collision_check as colli
from dataclasses import dataclass
import time


import onnx
import onnxruntime as ort
import copy

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
        self. qosProfileGen()
        self.response_timestamp = 0
        self.requestFlag = False
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.requestTimestamp = 0
        self.start_point = []
        self.goal_point = []
        self.waypoint_x = []
        self.waypoint_y = []
        self.waypoint_lenth = 0
        self.SACmoduleService_ = self.create_service(PathPlanningSetpoint, 'path_planning', self.PathPlanningCallback)
        print("=== Path Planning Node is Running =====")
        
    def PathPlanningCallback(self, request, response):
        print("===== Request PathPlanning Node =====")
        self.requestFlag = request.request_pathplanning
        self.requestTimestamp = request.request_timestamp
        if self.requestFlag is True : 
            self.Image= cv2.imread("/root/ros_ws/src/a4vai/a4vai/path_planning/Map/RawImage.png")
            self.start_point = request.start_point
            self.goal_point = request.goal_point
            # self.waypoint_x, self.waypoint_y, self.waypoint_lenth = SAC.PathPlanning(self, self.Image, self.start_point, self.goal_point)
            print("===== PathPlanning Complete!! =====")
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = True
            response.waypoint_x = self.waypoint_x
            response.waypoint_y = self.waypoint_y
            print("===== Response PathPlanning Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = False
            response.waypoint_x = [0] * 5
            response.waypoint_y = [0] * 5
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
        onnx_model = onnx.load('/root/ros_ws/src/a4vai/a4vai/path_planning/model/test28.onnx')
        onnx.checker.check_model(onnx_model)
        ort_session = ort.InferenceSession('/root/ros_ws/src/a4vai/a4vai/path_planning/model/test28.onnx')

        MapSize = 5000
        ############### Map 회전 방향 확인 필요 ################
        RawImage = Map

        Image = np.uint8(np.uint8((255 - RawImage)/ 255))
        Image_New = np.zeros((MapSize, MapSize))
        
        Start1 = time.time()
        ## Range [-2500, 2500]으로 바꾸기
        Step_Num = 5000

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
                for k in range(1, 250):
                    LidarState = LidarState + 1.0 * np.array([np.cos( ((30)*(j)) * np.pi / 180), 0, np.sin( ((30)*(j)) * np.pi / 180)])  # Radian으로 해야 함

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

            #Obs_mat = np.append(Obs_mat, Obs, axis=0)

            #if i == 1:
                #Act = 0
            #else:
            Act = ort_session.run(None, {"obs_0": Obs.astype(np.float32)})
            ## Make Move
            Act = Act[2][0][0]  # np.linalg.norm(Act[2][0] - Act[2][1])

            for j in range(0, 5):  # 0, 1, 2, 3, 4
                LOS_2D_Norm = (Target - Pos) / np.linalg.norm(Target - Pos)
                Action_Vec = np.array([LOS_2D_Norm[2], 0, -LOS_2D_Norm[0]])
                Vel = 5 * LOS_2D_Norm  # Original 5
                Action_Vec = 5 * Act * 2 * Action_Vec
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

        ## End SAC Computation
        End1 = time.time()

        for l in range(0,i):  # 0 - 4999
            path_x = np.append(path_x, Waypoint[l][0])
            path_y = np.append(path_y, Waypoint[l][2])

        ## Plot and Save Image
        imageLine = RawImage.copy()

        # 이미지에 맞게 pruned up Waypoint 변경 후 그리기
        for m in range(0, i-2):
            Im_i = int(path_x[m+1])
            Im_j = MapSize - int(path_y[m+1])

            Im_iN = int(path_x[m+2])
            Im_jN = MapSize - int(path_y[m+2])

            cv2.line(imageLine, (Im_i, Im_j), (Im_iN, Im_jN), (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        cv2.imwrite('/Result/TestResult0.png', imageLine)  ################################
        
        print(path_x)

        # For Pruning
        #def pruning(wp_epi, obsPos_epi, obsRad_epi, pruning_rate, margin): # pruning_rate[%], margin[%]
        # [wp_epi, obsPos_epi, obsRad_epi] = data()

        pruning_rate = 80

        if pruning_rate >= 100:
            pruning_rate = 99.99999

        ## Start Pruning Computation
        Start2 = time.time()

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

                # InitPos = np.array([wp_candidate[0], wp_candidate[1]])  # wp_candidate 초기 위치
                # FinalPos = np.array([origin_wp[i][0], origin_wp[i][1]])
                FinalPos = np.array([wp_candidate[0], wp_candidate[1]])  # wp_candidate 초기 위치
                InitPos = np.array([origin_wp[i][0], origin_wp[i][1]])
                # if (np.linalg.norm(FinalPos - InitPos) == 0):
                    
                #     break
                # else:           
                #     Unit = (FinalPos - InitPos) / np.linalg.norm((FinalPos - InitPos))
                FinalPosup = np.array([FinalPos[0]-1,FinalPos[1]+1])
                FinalPosdown = np.array([FinalPos[0]+1,FinalPos[1]-1])
                FinalPosUp = np.array([FinalPos[0],FinalPos[1]+1])
                FinalPosDown = np.array([FinalPos[0]+1,FinalPos[1]])
                Unit1 = (FinalPos - InitPos) / np.linalg.norm((FinalPos - InitPos))
                Unit2 = (FinalPosup - InitPos) / np.linalg.norm((FinalPosup - InitPos))
                Unit3 = (FinalPosdown - InitPos) / np.linalg.norm((FinalPosdown - InitPos))
                Unit4 = (FinalPosUp - InitPos) / np.linalg.norm((FinalPosUp - InitPos))
                Unit5 = (FinalPosDown - InitPos) / np.linalg.norm((FinalPosDown - InitPos))

                if i == 1300:
                    aa = 1

                CurrPos1 = InitPos
                CurrPos2 = InitPos
                CurrPos3 = InitPos
                CurrPos4 = InitPos
                CurrPos5 = InitPos
                for j in range(0, 600):
                    CurrPos1 = CurrPos1 + 0.5 * Unit1
                    CurrPos2 = CurrPos2 + 0.5 * Unit2
                    CurrPos3 = CurrPos3 + 0.5 * Unit3
                    CurrPos4 = CurrPos4 + 0.5 * Unit4
                    CurrPos5 = CurrPos5 + 0.5 * Unit5

                    if CurrPos1[0] > MapSize or CurrPos1[1] > MapSize:
                        collisionCheck = True
                        break
                    if CurrPos2[0] > MapSize or CurrPos2[1] > MapSize:
                        collisionCheck = True
                        break
                    if CurrPos3[0] > MapSize or CurrPos3[1] > MapSize:
                        collisionCheck = True
                        break
                    if CurrPos4[0] > MapSize or CurrPos4[1] > MapSize:
                        collisionCheck = True
                        break
                    if CurrPos5[0] > MapSize or CurrPos5[1] > MapSize:
                        collisionCheck = True
                        break

                    if CurrPos1[0] < 0 or CurrPos1[1] < 0:
                        collisionCheck = True
                        break
                    if CurrPos2[0] < 0 or CurrPos2[1] < 0:
                        collisionCheck = True
                        break
                    if CurrPos3[0] < 0 or CurrPos3[1] < 0:
                        collisionCheck = True
                        break
                    if CurrPos4[0] < 0 or CurrPos4[1] < 0:
                        collisionCheck = True
                        break
                    if CurrPos5[0] < 0 or CurrPos5[1] < 0:
                        collisionCheck = True
                        break

                    # if np.isnan(CurrPos1[0]) or np.isnan(CurrPos1[1]):
                    #     collisionCheck = True
                    #     break
                    
                    if Image_New[int(CurrPos1[1])][int(CurrPos1[0])] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    if Image_New[int(CurrPos1[1]+1)][int(CurrPos1[0])+1] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    if Image_New[int(CurrPos1[1]-1)][int(CurrPos1[0])-1] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    if Image_New[int(CurrPos2[1])][int(CurrPos2[0])] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    if Image_New[int(CurrPos3[1])][int(CurrPos3[0])] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    if Image_New[int(CurrPos4[1])][int(CurrPos4[0])] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    if Image_New[int(CurrPos5[1])][int(CurrPos5[0])] > 0:  # 꼭 후처리된 Map에서 # 장애물에 부딪침
                        collisionCheck = True
                        break
                    
                    if wp_candidate == origin_wp[len(origin_wp) - 1] and InitPos[0] == origin_wp[0][0]:  # 마지막 WP
                        collisionCheck = True
                        break
                    
                    if np.linalg.norm(CurrPos1 - FinalPos) < 10:  # 장애물 없이 다음 WP까지 가면 반복문 나오기(collisionCheck 0으로 해서)
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

        ## Plot
        pruned_x_points = []
        pruned_y_points = []
        for i in range(len(pruned_wp)):
            pruned_x_points.append(pruned_wp[i][0])
            pruned_y_points.append(pruned_wp[i][1])

        ## End Pruning Computation
        End2 = time.time()

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

        return path_x, path_y, len(pruned_wp)
