import sys
import time
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

# from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import EstimatorStates
from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingSetpoint


class PFAttitudeCmdModule(Node):
    def __init__(self):
        super().__init__('pf_attitude_cmd_module')
        ##  Input
        self.requestFlag = False    #   bool
        self.requestTimestamp = 0   #   uint
        self.PlannedX = []  #   double
        self.PlannedY = []  #   double
        self.PlannedZ = []  #   double
        self.PlannedIndex = 0   #   int
        self.Pos = []   #   double
        self.Vn = []    #   double
        self.AngEuler = []  #   double
        self.Acc_disturb = []   #   double
        self.LAD = 0    #   double      Least absolute deviation ???
        self.SPDCMD = 0 #   double
        ##  Output
        self.response_timestamp = 0 #   uint
        self.TargetThrust = 0   #   
        self.TargetAttitude = []    #   double
        self.TargetPosition = []    #   double
        self.TargetYaw = 0
        self.outNDO = []    #   double
        ##  Function
        self.qosProfileGen()
        self.declare_subscriber_px4()
        self.PFAttitudeCmdService_ = self.create_service(PathFollowingSetpoint, 'path_following_att_cmd', self.PFAttitudeCmdCallback)
        print("===== Path Following Attitude Command Node is Initialize =====")

#################################################################################################################

    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        # self.VehicleAngularVelocitySubscriber_ = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.VehicleAngularVelocityCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")
        
        
    def PFAttitudeCmdCallback(self, request, response):
        print("===== Request Path Following Attitude Command Node =====")
        '''
        uint64 request_timestamp	# time since system start (microseconds)
        bool request_pathfollowing
        float64[] waypoint_x
        float64[] waypoint_y
        float64[] waypoint_z
        uint32 waypoint_index
        float64 lad
        float64 spd_cmd
        '''
        self.requestFlag = request.request_pathfollowing
        self.requestTimestamp = request.request_timestamp
        self.PlannedX = request.waypoint_x
        self.PlannedY = request.waypoint_y
        self.PlannedZ = request.waypoint_z
        self.PlannedIndex = request.waypoint_index
        self.LAD = request.lad
        self.SPDCMD = request.spd_cmd
        if self.requestFlag is True : 
            ##  Algorithm Function
            '''
            uint64 response_timestamp	# time since system start (microseconds)
            bool response_pathfollowing
            float64 targetthrust
            float64[] targetattitude
            float64[] targetposition
            float64 targetyaw
            float64[] outndo
            '''
            print("===== Path Following Attitude Command Generation !! =====")
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = True
            response.targetthrust = self.TargetThrust
            response.targetattitude = self.TargetAttitude
            response.targetposition = self.TargetPosition
            response.targetyaw = self.TargetYaw
            response.out_ndo = self.outNDO
            print("===== Response Path Following Attitude Command Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_pathplanning = True
            response.targetthrust = self.TargetThrust
            response.targetattitude = self.TargetAttitude
            response.targetposition = self.TargetPosition
            response.targetyaw = self.TargetYaw
            response.out_ndo = self.outNDO
            print("===== Can't Response Path Following Attitude Command Node =====")
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
    
    # # VehicleAngularVelocity
    # def VehicleAngularVelocityCallback(self, msg):
    #     # Rate
    #     self.p = msg.xyz[0] * 57.295779513
    #     self.q = msg.xyz[1] * 57.295779513
    #     self.r = msg.xyz[2] * 57.295779513
        
    def Quaternion2Euler(self, w, x, y, z):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        Roll = math.atan2(t0, t1) * 57.2958

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Pitch = math.asin(t2) * 57.2958

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Yaw = math.atan2(t3, t4) * 57.2958

        return Roll, Pitch, Yaw