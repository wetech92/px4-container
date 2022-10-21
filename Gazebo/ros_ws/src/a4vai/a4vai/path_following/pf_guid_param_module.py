import sys
import time
from tkinter import E
import matplotlib.pyplot as plt
import numpy as np
import math

import time


import onnx
import onnxruntime as ort


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

from px4_msgs.msg import EstimatorStates
from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingGuid

class PFGuidModule(Node):
    def __init__(self):
        super().__init__('pf_guid_module')
        ##  Input
        self.PlannedX = []  #   double
        self.PlannedY = []  #   double
        self.PlannedZ = []  #   double
        self.PlannedIndex = 0   #   int
        self.Pos = []   #   double
        self.Vn = []    #   double
        self.AngEuler = []  #   double
        self.GPR_output = []    #   double
        self.outNDO = []    #   double
        self.Flag_Guid_Param = 0    #   int
        ##  Output
        self.response_timestamp = 0 #   uint
        self.LAD = 0    #   double
        self.SPDCMD = 0 #   double
        ##  Function
        self.qosProfileGen()
        self.declare_subscriber_px4()
        self.PFGuidService_ = self.create_service(PathFollowingGuid, 'path_following_guid', self.PFGuidCallback)
        print("===== Path Following Guidance Node is Initialize =====")
        
        
    def PFGuidCallback(self, request, response):
        print("===== Request Path Following Guidance Node =====")
        '''
        uint64 request_timestamp	# time since system start (microseconds)
        bool request_guid
        float64[] waypoint_x
        float64[] waypoint_y
        float64[] waypoint_z
        uint32 waypoint_index
        float64[] out_ndo
        int32 flag_guid_param
        ---
        bool response_guid
        float64 lad
        float64 spdcmd
        '''
        self.requestFlag = request.request_guid
        self.requestTimestamp = request.request_timestamp
        self.PlannedX = request.waypoint_x
        self.PlannedY = request.waypoint_y
        self.PlannedZ = request.waypoint_z
        self.PlannedIndex = request.waypoint_index
        self.outNDO = request.out_ndo
        self.Flag_Guid_Param = request.flag_guid_param
        
        if self.requestFlag is True : 
            
            print("===== Path Following Guidance Generation !! =====")
            
            response.response_timestamp = self.response_timestamp
            response.response_guid = True
            response. lad = self.LAD
            print("===== Response Path Following Guidance Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_guid = False
            response.
            print("===== Can't Response Path Following Guidance Node =====")
            return response

        
    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        self.VehicleAngularVelocitySubscriber_ = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.VehicleAngularVelocityCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")
        
        
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
        
    def EstimatorStatesCallback(self, msg):
        #   TimeStamp
        self.EstimatorStatesTime = msg.timestamp
        #   Position NED
        self.x = msg.states[7]
        self.y = msg.states[8]
        self.z = msg.states[9]
        #   Velocity NED
        self.vx = msg.states[4]
        self.vy = msg.states[5]
        self.vz = msg.states[6]
        #   Attitude
        self.roll, self.pitch, self.yaw = self.Quaternion2Euler(msg.states[0], msg.states[1], msg.states[2], msg.states[3])
        #   Wind Velocity NE
        self.wn = msg.states[22]
        self.we = msg.states[23]
        
        self.Pos = [self.x, self.y, self.z]
        self.Vn = [self.vx, self.vy, self.vz]
        self.AngEuler = [self.roll, self.theta, self.psi]
        