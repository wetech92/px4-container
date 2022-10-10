import matplotlib.pyplot as plt
import numpy as np
import collections
import math
from .collision_check import collision_check as colli
from dataclasses import dataclass
import time

import onnx 
import onnxruntime
import sys

# Opencv-ROS
from cv_bridge import CvBridge
import cv2

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

class SACOnnx:

    def PathPlanning(self, Image, Start, Goal) :

        onnx_model = onnx.load('/root/ros_ws/src/integration/integration/PathPlanning/SAC/test26.onnx')
        onnx.checker.check_model(onnx_model)
        ort_session = onnxruntime.InferenceSession('/root/ros_ws/src/integration/integration/PathPlanning/SAC/test26.onnx')

        # 100 x 100 Drone.onnx를 위한 환경
        # 1. Input Matrix 만들기, 2. Onnx Model에 넣기(맞는 위치에 저장) 3. Output 형식 확인
        step_num = 5000
        Map_Size = Image.shape[0]
        ScaleFactor = 5000/Map_Size

        #init = np.array([0, 5, 0])
        #final = np.array([100, 5, 100])  # 수정 예정
        
        init = np.array([Start[0]/ScaleFactor, 5, Start[1]/ScaleFactor], dtype=float)
        final = np.array([Goal[0]/ScaleFactor, 5, Goal[1]/ScaleFactor], dtype=float)

        print(Image.shape[0])
        #print(Start[0]/ScaleFactor)
        #print(Goal[0]/ScaleFactor)
        print(init)
        print(final)

        ## Target을 향한 Unit Veoctor로 가고 있다고 가정
        # Generate Current State Matrix
        CurrState_mat = np.zeros((step_num,3))
        CurrState_mat[0] = init
        CurrState_temp = init
        for i in range(1,step_num):  # 1부터 시작
            CurrState_temp = CurrState_temp + (final-CurrState_temp)/np.linalg.norm(final-CurrState_temp)
            CurrState_mat[i] = CurrState_temp

        ## Input 1 : Target - 현재 위치(3 axis)(Unit Vector), Input 2 : LOS 거리(3 axis), Input 3 : Lidar (12개)
        input1_mat = np.zeros((step_num,3))
        input2_mat = np.zeros((step_num,3))
        input3_mat = np.zeros((step_num,12))
        for i in range(0,step_num):  # 0 - step_num
            input1_mat[i] = (final - CurrState_mat[i])/np.linalg.norm(final - CurrState_mat[i])
            
            LOS_x = final[0] - CurrState_mat[i][0]
            LOS_z = final[2] - CurrState_mat[i][2]
            input2_mat[i] = np.array([LOS_x, 0, LOS_z])

        ## Imput 3 Data 만들기    
        RawImage = (cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/test.png", cv2.IMREAD_GRAYSCALE))
        Image = np.uint8(np.uint8((255 - RawImage)/ 255))
        Image = cv2.flip(Image,0)
        Image = cv2.rotate(Image, cv2.ROTATE_90_CLOCKWISE)

        LidarObs = np.zeros((step_num,12))  # (Iteration, Lidar_Index)
        LidarObs_mat = np.zeros((step_num,1))

        for k in range(0,step_num):  # 0 - (step_num-1)
    
            for i in range(0,12):  # 0 - 11
                CurrState = CurrState_mat[k]
                LidarState = CurrState
        
            for j in range(0,step_num):  # step_num개 (0 - (step_num)))
                LidarState = LidarState + [np.cos(30*i*np.pi/180), 0, np.sin(30*i*np.pi/180)]  # Radian으로 해야 함
            
                # Map Size 넘어갈만큼 진전시켰으면 Data 수집 멈추기
                if LidarState[0] >= Map_Size:  # 여기는 step_num이 아니라 Map Size로 해야 함
                    break
                if LidarState[2] >= Map_Size:
                    break
            
                if Image[ int(LidarState[0]) ][ int(LidarState[2]) ] == 1:
                    Diff = LidarState - CurrState
                    LidarObs_mat[j] = np.linalg.norm(Diff)
                else:
                    LidarObs_mat[j] = step_num *  2  # 일단 제일 크게
        
            LidarObs[k][i] = min(LidarObs_mat)

        input3_mat = LidarObs

        ## Make Observation Matrix and Execute Onnx Model
        input_mat = np.concatenate((input1_mat,input2_mat), axis=1)
        input_mat = np.concatenate((input_mat,input3_mat), axis=1)
        input_mat = input_mat.astype(np.float32)  # Change array type

        output_mat = ort_session.run(None, {ort_session.get_inputs()[0].name : input_mat})

        ## Make Waypoint Matrix
        Waypoint_mat = np.zeros((step_num,3))
        Waypoint_mat[0] = CurrState_mat[0]
        for i in range(1,step_num):
            Act_mag = abs(output_mat[2][i-1])  # onnx ouput의 2항이 Action 크기
            Act_dir_rel = (final-CurrState_mat[i-1])/np.linalg.norm(final-CurrState_mat[i-1])  # 매 순간 Target을 향하는 Unit Vector가 Action 상대 방향
            if output_mat[2][i-1][0] >= 0:  # 일단 고도 일정한 2차원에서만 적용되도록  # (x, 2, y)  # z쪽으로는 상대 단위벡터 없음  # Action 뭔지 확인하기!!(일단 첫번쨰 값으로)
                Act_dir = np.array([-Act_dir_rel[2],0,Act_dir_rel[0]])  # 매 순간 상대 방향으로 Action 방향 구하기
            else:
                Act_dir = np.array([Act_dir_rel[2],0,-Act_dir_rel[0]])
            
            Act_temp = Act_mag*Act_dir

            Waypoint_temp = CurrState_mat[i] + Act_temp  # 해당 Step에서 Act_temp 추가시킴
            Waypoint_mat[i] = Waypoint_temp

        ScaleFactor = 5000/Map_Size

        path_x = np.array([])
        path_y = np.array([])

        for i in range(0,step_num):
            path_x = np.append(path_x, Waypoint_mat[i][0] * ScaleFactor)
            path_y = np.append(path_y, Waypoint_mat[i][2] * ScaleFactor)


        return path_x, path_y
