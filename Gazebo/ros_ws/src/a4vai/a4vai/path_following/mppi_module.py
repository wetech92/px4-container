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

from .mppi.PF import PF
from .mppi.NDO import NDO
from .gpr.GPR import GPR
from .mppi.Guid_MPPI import MPPI
from .mppi.PF_Cost import Calc_PF_cost


from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingSetpoint


# Opencv-ROS
import cv2


class MPPINode(Node):
    def __init__(self):
        super().__init__('mppi_module')
        self.response_timestamp = 0
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.PlannedX = []
        self.PlannedY = []
        self.phi_cmd = 0
        self.theta_cmd = 0
        self.psi_cmd = 0
        self.psi_cmd_prev = 0
        self.requestFlag = False
        self. qosProfileGen()
        self.KAISTmoduleService_ = self.create_service(PathFollowingSetpoint, 'path_following', self.PathFollowingCallback)

        print("=== Path Following Node is Running =====")

        
    def PathFollowingCallback(self, request, response):
        print("===== Request PathFollowing Node =====")
        self.requestFlag = request.request_pathfollowing
        self.requestTimestamp = request.request_timestamp
        if self.requestFlag is True : 
            self.PlannedX = request.waypoint_x
            self.PlannedY = request.waypoint_y
            
            print("===== PathFollowing Complete!! =====")
            
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = True
            response.phi_cmd = self.phi_cmd
            response.theta_cmd = self.theta_cmd
            response.psi_cmd = self.psi_cmd
            print("===== Response PathPlanning Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = False
            response.phi_cmd = 0
            response.theta_cmd = 0
            response.psi_cmd = 0
        
        
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
        
    
    def KAIST_MPPI_CallBack(self):
        if self.InitialPositionFlag and self.PFmoduleCount > 50:
            self.t.tic()
            if self.MPPI.MPPIParams.count % self.MPPI.MPPIParams.UpdateCycle == 0:
                if self.Flag_UseGPR == 1:
                    self.MPPI.MPPIParams.est_delAccn    =   self.GPR.yPred
                else:
                    self.MPPI.MPPIParams.est_delAccn    =   self.NDO.outNDO * np.ones((self.MPPI.MPPIParams.N, 3))

                Pos         =   np.array([self.x, self.y, self.z])
                Vn          =   np.array([self.vx, self.vy, self.vz])
                AngEuler    =   np.array([self.roll, self.pitch, self.yaw]) * math.pi /180.
                # function
                start = time.time()
                u1, u1_MPPI, u2_MPPI    =   self.MPPI.Guid_MPPI(self.PF.GCUParams, self.PF.WPs, Pos, Vn, AngEuler)
                if self.Flag_PrintMPPItime == 1 and self.PFmoduleCount < self.Flag_PrintLimitCount:
                    print("MPPI call. time :", round(self.CurrTime - self.InitTime, 6), ", calc. time :", round(time.time() - start, 4),", PFmoduleCount :", self.PFmoduleCount)
            
                #.. Limit
                Kmin    =   self.MPPI.MPPIParams.u1_min
                LADmin  =   self.MPPI.MPPIParams.u2_min
                u1_MPPI     =   np.where(u1_MPPI < Kmin, Kmin, u1_MPPI)
                u2_MPPI     =   np.where(u2_MPPI < LADmin, LADmin, u2_MPPI)

                # output
                tau_u       =   self.MPPI.MPPIParams.tau_LPF
                N_tau_u     =   self.MPPI.MPPIParams.N_tau_LPF
                
                for i_u in range(self.MPPI.MPPIParams.N - 1):
                    du1     =   1/tau_u * (u1_MPPI[i_u + 1] - u1_MPPI[i_u])
                    u1_MPPI[i_u + 1] = u1_MPPI[i_u] + du1 * self.MPPI.MPPIParams.dt_MPPI
                    du2     =   1/tau_u * (u2_MPPI[i_u + 1] - u2_MPPI[i_u])
                    u2_MPPI[i_u + 1] = u2_MPPI[i_u] + du2 * self.MPPI.MPPIParams.dt_MPPI
                
                for i_N in range(N_tau_u):
                    u1_MPPI[0:self.MPPI.MPPIParams.N - 1] = u1_MPPI[1:self.MPPI.MPPIParams.N]
                    u1_MPPI[self.MPPI.MPPIParams.N - 1]  = 0.5 * (np.max(u1_MPPI) + np.min(u1_MPPI))
                    u2_MPPI[0:self.MPPI.MPPIParams.N - 1] = u2_MPPI[1:self.MPPI.MPPIParams.N]
                    u2_MPPI[self.MPPI.MPPIParams.N - 1]  = 0.5 * (np.max(u2_MPPI) + np.min(u2_MPPI))

                self.MPPI.MPPIParams.u1_MPPI = u1_MPPI
                self.MPPI.MPPIParams.u2_MPPI = u2_MPPI

            u1_MPPI     =   self.MPPI.MPPIParams.u1_MPPI
            u2_MPPI     =   self.MPPI.MPPIParams.u2_MPPI

            #.. direct
            u1_MPPI[0:self.MPPI.MPPIParams.N - 1] = u1_MPPI[1:self.MPPI.MPPIParams.N]
            u1_MPPI[self.MPPI.MPPIParams.N - 1]  = 0.5 * (np.max(u1_MPPI) + np.min(u1_MPPI))
            u2_MPPI[0:self.MPPI.MPPIParams.N - 1] = u2_MPPI[1:self.MPPI.MPPIParams.N]
            u2_MPPI[self.MPPI.MPPIParams.N - 1]  = 0.5 * (np.max(u2_MPPI) + np.min(u2_MPPI))

            # update
            self.MPPI.MPPIParams.u1_MPPI     =   u1_MPPI
            self.MPPI.MPPIParams.u2_MPPI     =   u2_MPPI

            # output
            # self.PF.GCUParams.Kgain_guidPursuit    =   u1[1]
            self.PF.GCUParams.desSpd                =   u1_MPPI[0]
            self.PF.GCUParams.lookAheadDist         =   u2_MPPI[0]
            self.PF.GCUParams.reachDist             =   self.PF.GCUParams.lookAheadDist

            self.MPPI.MPPIParams.count = self.MPPI.MPPIParams.count + 1
            self.t.toc()
            
            
        pass

    ## GPR_Update_CallBack
    def KAIST_GPR_Update_CallBack(self):
        if self.InitialPositionFlag and self.OffboardCount > 0:
            
            x_new   =   self.PF.GCUTime
            Y_new   =   self.NDO.outNDO
            self.GPR.GPR_dataset(x_new,Y_new)

            if self.GPR.count % self.GPR.EstimateCycle == 0:
            #.. GPR Estimation
                self.GPR.GPR_estimate(x_new,testSize=self.GPR.N,dt=self.GPR.dt_Est)

            if self.GPR.count % self.GPR.UpdateCycle == 0:
            #.. GPR Update
                self.GPR.GPR_update()

            self.GPR.count = self.GPR.count + 1
            
        pass