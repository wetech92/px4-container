import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from itertools import chain

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
from px4_msgs.msg import EstimatorStates


from .PathFollowing.PF_GPR import PF_GPR

from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingGPR
from msg_srv_act_interface.msg import PFGpr2PFGuid
from msg_srv_act_interface.msg import PFAtt2PFGuid


class PF_GPR_Module(Node):
    def __init__(self):
        super().__init__('pf_gpr_module')
        ##  Input
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
        
        self.EstimatorStatesTime = 0
        self.InitTime = 0
        self.InitFlag = True
        self.requestFlag = False    #   bool
        self.requestTimestamp = 0   #   uint
        self.outNDO = [0.0, 0.0, 0.0]    #   double
        ##  Output
        self.response_timestamp = 0 #   uint
        self.gpr_output_data = []
        self.gpr_output = []
        self.gpr_output_index = 0
        self.PF_GPR_ = PF_GPR()
        self.PF_GPR_Period = 8/100
        ##  Function
        self.qosProfileGen()
        self.declare_subscriber_px4()
        ##  Pub
        self.PF_Gpr2PF_Guid_Publisher_ = self.create_publisher(PFGpr2PFGuid, '/pf_gpr_2_pf_guid', self.QOS_Sub_Sensor)
        ##  Sub
        self.PF_Att2PF_Gpr_Subscriber_ = self.create_subscription(PFAtt2PFGuid, '/pf_att_2_pf_guid', self.PF_Att2PF_Gpr_callback, self.QOS_Sub_Sensor)
        
        self.PFGPRService_ = self.create_service(PathFollowingGPR, 'path_following_gpr', self.PFGPRSRVCallback)
        print("===== Path Following Attitude Command Node is Initialize =====")
        
#################################################################################################################
    def PFGPRCallback(self):
        ############# Algirithm Start - PF_GPR  #############
        if self.InitFlag is True:
            self.InitTime  =   self.EstimatorStatesTime * 10**(-6)
            self.InitFlag   =   False
        Time   =   self.EstimatorStatesTime * 10**(-6) - self.InitTime
        # print("Time = ", str(Time))
        # print("outNDO = ", str(self.outNDO))
        # function
        GPR_out = self.PF_GPR_.PF_GPR_Module(Time, self.outNDO)
        # output
        self.gpr_output = GPR_out.copy()
        ############# Algirithm  End  - PF_GPR  #############
        self.gpr_output_data = list(chain.from_iterable(self.gpr_output))
        self.gpr_output_index = 3
        self.PF_Gpr2PF_Guid_Publisher()
    
    def PF_Gpr2PF_Guid_Publisher(self):
        msg = PFGpr2PFGuid()
        msg.timestamp = self.response_timestamp
        msg.gpr_output_data = self.gpr_output_data
        msg.gpr_output_index = self.gpr_output_index
        self.PF_Gpr2PF_Guid_Publisher_.publish(msg)
        
    def PF_Att2PF_Gpr_callback(self, msg):
        self.outNDO = msg.out_ndo
    
    def PFGPRSRVCallback(self, request, response):
        '''
        #####   SRV
        uint64 request_timestamp	# time since system start (microseconds)
        bool request_gpr
        #####   MSG
        float64[] out_ndo
        '''
        self.requestFlag = request.request_gpr
        self.requestTimestamp = request.request_timestamp
        if self.requestFlag is True : 
            self.PF_GPR_TIMER = self.create_timer(self.PF_GPR_Period, self.PFGPRCallback)
            # print("Create Timer")
            '''
            #####   SRV
            uint64 response_timestamp	# time since system start (microseconds)
            bool response_gpr
            #####   MSG
            float64[] gpr_output_data
            uint32 gpr_output_index
            '''
            response.response_timestamp = self.response_timestamp
            response.response_gpr = True
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_gpr = False
            return response

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
        
    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")
        
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
        Roll = math.atan2(t0, t1) * 57.2958

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Pitch = math.asin(t2) * 57.2958

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Yaw = math.atan2(t3, t4) * 57.2958

        return Roll, Pitch, Yaw
        
def main(args=None):
    rclpy.init(args=args)
    
    pf_gpr_module = PF_GPR_Module()

    try : 
        rclpy.spin(pf_gpr_module)
    except Exception as e:
                    pf_gpr_module.get_logger().info(
                        'GPR module Start failed %r' % (e,))
    finally :
        pf_gpr_module.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
