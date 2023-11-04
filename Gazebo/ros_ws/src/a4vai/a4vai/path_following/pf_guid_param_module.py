# from tkinter import E
import matplotlib.pyplot as plt
import numpy as np
import time

# pulbic libs.
import math as m
import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule


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

from .PathFollowing.PF_GUID_PARAM import PF_GUID_PARAM

from px4_msgs.msg import EstimatorStates
from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingGuid
from msg_srv_act_interface.msg import WayPointIndex
from msg_srv_act_interface.msg import PFGuid2PFAtt
from msg_srv_act_interface.msg import PFAtt2PFGuid
from msg_srv_act_interface.msg import PFGpr2PFGuid

class PFGuidModule(Node):
    def __init__(self):
        super().__init__('pf_guid_module')
        ## Input
        self.EstimatorStatesTime = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        
        self.wn = 0.0
        self.we = 0.0
        
        self.Pos = [self.x, self.y, self.z]
        self.Vn = [self.vx, self.vy, self.vz]
        self.AngEuler = [self.phi, self.theta, self.psi]
        self.waypoint_index = 0
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
        self.GPR_output_data = []
        self.GPR_output_index = 3
        self.GPR_output = []    #   double
        self.outNDO = []    #   double
        self.Flag_Guid_Param = 2    #   int
        ##  Output
        self.response_timestamp = 0 #   uint
        self.LAD = 0.0    #   double
        self.SPDCMD = 0.0 #   double
        
        self.PF_GUID_Period = 8/100
        ############# Variables Start - PF_GUID_PARAM  #############
        # Parameter Decision of the Guidance Law - Flag 0 : MPPI / Flag 1 : NM / Flag 2 : Constant Parameter
        # self.PF_GUID_PARAM_MOD   =   PF_GUID_PARAM(0)
        self.InitFlag   =   True
        ############# Variables  End  - PF_GUID_PARAM  #############

        
        ##  Function
        self.qosProfileGen()
        self.declare_subscriber_px4()
        self.PFGuidService_ = self.create_service(PathFollowingGuid, 'path_following_guid', self.PFGuidSRVCallback)
        ######  PUB
        self.Waypoint_Indx_Publisher_ = self.create_publisher(WayPointIndex, '/waypoint_indx', self.QOS_Sub_Sensor)
        self.PF_Guid2PF_Att_Publisher_ = self.create_publisher(PFGuid2PFAtt, '/pf_guid_2_pf_att', self.QOS_Sub_Sensor)
        ######
        self.PF_Att2PF_GuidSubscriber_ = self.create_subscription(PFAtt2PFGuid, '/pf_att_2_pf_guid', self.PF_Att2PF_Guid_callback, self.QOS_Sub_Sensor)
        self.PF_Gpr2PF_GuidSubscriber_ = self.create_subscription(PFGpr2PFGuid, '/pf_gpr_2_pf_guid', self.PF_GPR2PF_Guid_callback, self.QOS_Sub_Sensor)
        print("===== Path Following Guidance Node is Initialize =====")
        
#################################################################################################################

    def waypoint_index_calculator(self):
        if np.sqrt((self.x - self.PlannedX[self.PlannedIndex])**2 + (self.y - self.PlannedY[self.PlannedIndex]) ** 2) < 3.0:
            self.PlannedIndex += 1
            # print("Waypoint index ++")
        else : 
            pass
        
    def Waypoint_indexPublisher(self):
        msg = WayPointIndex()
        msg.timestamp = self.response_timestamp
        msg.waypoint_index = self.PlannedIndex
        self.Waypoint_Indx_Publisher_.publish(msg)
        
    def PF_GUID_2_PF_ATT_Publisher(self):
        msg = PFGuid2PFAtt()
        msg.timestamp = self.response_timestamp
        msg.lad = self.LAD
        msg.spd_cmd =self.SPDCMD
        self.PF_Guid2PF_Att_Publisher_.publish(msg)
        
    def PF_Att2PF_Guid_callback(self, msg):
        self.outNDO = msg.out_ndo
        
    def PF_GPR2PF_Guid_callback(self, msg):
        self.GPR_output_data = msg.gpr_output_data
        self.GPR_output_index = msg.gpr_output_index
        
    def PFGuidCallback(self):
        if self.requestFlag is True :
            self.waypoint_index_calculator()
            # print("flag = ",str(self.Flag_Guid_Param))
            
            self.GPR_output = [self.GPR_output_data[i * self.GPR_output_index:(i + 1) * self.GPR_output_index] for i in range((len(self.GPR_output_data) - 1 + self.GPR_output_index) // self.GPR_output_index )]
            if self.InitFlag is True:
                self.PF_GUID_PARAM_MOD  =  PF_GUID_PARAM(self.Flag_Guid_Param)
                self.InitFlag = False
            else : 
                pass
            Pos         =   [self.x, self.y, self.z]
            Vn          =   [self.vx, self.vy, self.vz]
            AngEuler    =   [self.phi * m.pi /180., self.theta * m.pi /180., self.psi * m.pi /180.]
            # function
            LAD, SPDCMD = self.PF_GUID_PARAM_MOD.PF_GUID_PARAM_Module(
                self.PlannedX, self.PlannedY, self.PlannedZ, self.PlannedIndex, Pos, Vn, AngEuler, self.GPR_output, self.outNDO, self.Flag_Guid_Param)
            # output
            self.LAD    =   LAD
            self.SPDCMD =   SPDCMD
            self.Waypoint_indexPublisher()
            self.PF_GUID_2_PF_ATT_Publisher()
        else : 
            pass
        
    def PFGuidSRVCallback(self, request, response):
        print("===== Request Path Following Guidance Node =====")
        '''
        ####    SRV
        uint64 request_timestamp	# time since system start (microseconds)
        bool request_guid
        float64[] waypoint_x
        float64[] waypoint_y
        float64[] waypoint_z
        int32 flag_guid_param
        ####    MSG
        float64[] gpr_output_data
        uint32 gpr_output_index
        
        float64[] out_ndo
        '''
        self.requestFlag = request.request_guid
        self.requestTimestamp = request.request_timestamp
        self.PlannedX = request.waypoint_x
        self.PlannedY = request.waypoint_y
        self.PlannedZ = request.waypoint_z
        # #### TODO
        # self.PlannedX[0] = 2.0
        # self.PlannedY[0] = 2.0
        self.Flag_Guid_Param = request.flag_guid_param
        if self.requestFlag is True : 
            self.PF_GUID_TIMER = self.create_timer(self.PF_GUID_Period, self.PFGuidCallback)
            ############# Algirithm  End  - PF_GUID_PARAM  #############
            print("===== Path Following Guidance Generation !! =====")
            '''
            ####    SRV
            uint64 response_timestamp	# time since system start (microseconds)
            bool response_guid
            ####    MSG
            float64 lad
            float64 spd_cmd
            '''
            response.response_timestamp = self.response_timestamp
            response.response_guid = True
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_guid = False
            return response
        
    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        # self.VehicleAngularVelocitySubscriber_ = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.VehicleAngularVelocityCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")
        
    def TimesyncCallback(self, msg):
        self.response_timestamp = msg.timestamp
        
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
        self.phi, self.theta, self.psi = self.Quaternion2Euler(msg.states[0], msg.states[1], msg.states[2], msg.states[3])
        #   Wind Velocity NE
        self.wn = msg.states[22]
        self.we = msg.states[23]
        
        self.Pos = [self.x, self.y, self.z]
        self.Vn = [self.vx, self.vy, self.vz]
        self.AngEuler = [self.phi, self.theta, self.psi]
        
        
    def Quaternion2Euler(self, w, x, y, z):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        Roll = m.atan2(t0, t1) * 57.2958

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Pitch = m.asin(t2) * 57.2958

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Yaw = m.atan2(t3, t4) * 57.2958

        return Roll, Pitch, Yaw


def main(args=None):
    rclpy.init(args=args)
    
    pf_guid_module = PFGuidModule()

    try :       
        rclpy.spin(pf_guid_module)
    except Exception as e:
                    pf_guid_module.get_logger().info(
                        'MPPI module Start failed %r' % (e,))
    finally :

        pf_guid_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
