
import matplotlib.pyplot as plt
import math
import numpy as np

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

from .PathFollowing.PF_ATTITUDE_CMD import PF_ATTITUDE_CMD

# from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import EstimatorStates
from px4_msgs.msg import Timesync
from msg_srv_act_interface.srv import PathFollowingSetpoint
from msg_srv_act_interface.msg import PFAtt2Control
from msg_srv_act_interface.msg import PFGuid2PFAtt
from msg_srv_act_interface.msg import PFAtt2PFGuid
from msg_srv_act_interface.msg import WayPointIndex

class PFAttitudeCmdModule(Node):
    def __init__(self):
        super().__init__('pf_attitude_cmd_module')
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
        self.requestInitTimestamp = 0   #   uint
        self.requestTimestamp = 0   #   uint
        self.PlannedX = []  #   double
        self.PlannedY = []  #   double
        self.PlannedZ = []  #   double
        self.PlannedIndex = 0   #   int
        self.Pos = []   #   double
        self.Vn = []    #   double
        self.AngEuler = []  #   double
        self.Acc_disturb = []   #   double
        self.LAD = 0.0    #   double      Least absolute deviation ???
        self.SPDCMD = 0.0 #   double
        self.z_NDO_past = []
        ##  Output
        self.response_timestamp = 0 #   uint
        self.TargetThrust = 0.0   #   
        self.TargetAttitude = []    #   double
        self.TargetPosition = []    #   double
        self.TargetYaw = 0.0
        self.outNDO = [0.0, 0.0, 0.0]    #   double
        self.z_NDO = []
        ##
        self.PF_ATT_Period = 1/250
        
        ##  Function
        self.qosProfileGen()
        self.declare_subscriber_px4()
        self.PFAttitudeCmdService_ = self.create_service(PathFollowingSetpoint, 'path_following_att_cmd', self.PF_ATT_SRV_Callback)
        self.PF_ATTITUDE_CMD_ = PF_ATTITUDE_CMD(0.004)
        
        self.First_Flag = True
        self.InitTime = 0
        ######  PUB
        self.PFAttToControlPublisher_ = self.create_publisher(PFAtt2Control, '/pf_att_2_control', self.QOS_Sub_Sensor)
        self.PFAttToPFGuidPublisher_ = self.create_publisher(PFAtt2PFGuid, '/pf_att_2_pf_guid', self.QOS_Sub_Sensor)
        ######  SUB
        self.PFGuidToPFAttSubscriber_ = self.create_subscription(PFGuid2PFAtt, '/pf_guid_2_pf_att', self.PF_Guid2PF_AttSubscribe, self.QOS_Sub_Sensor)
        self.WaypointIndexSubscriber_ = self.create_subscription(WayPointIndex, '/waypoint_indx', self.Waypoint_index_callback, self.QOS_Sub_Sensor)
        print("===== Path Following Attitude Command Node is Initialize =====")

#################################################################################################################


    def declare_subscriber_px4(self):
        #   init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        print("====== px4 Subscriber Open ======")
        
    def PFAttitudeCmdCallback(self):
        if self.requestFlag is True :
            if self.First_Flag is True :
                self.InitTime  =   self.EstimatorStatesTime * 10**(-6)
                self.First_Flag = False
            else : 
                Time   =   self.EstimatorStatesTime * 10**(-6) - self.InitTime 
                Pos         =   [self.x, self.y, self.z]
                Vn          =   [self.vx, self.vy, self.vz]
                AngEuler    =   [self.phi * math.pi /180., self.theta * math.pi /180., self.psi * math.pi /180.]
                Acc_disturb =   [0., 0., 0.]
        ### Todo kdh
                self.LAD = 2.0
                self.SPDCMD = 2.0
                self.TargetThrust, self.TargetAttitude, self.TargetPosition, self.TargetYaw, self.outNDO = \
                    self.PF_ATTITUDE_CMD_.PF_ATTITUDE_CMD_Module(Time, self.PlannedX, self.PlannedY, self.PlannedZ, self.PlannedIndex, Pos, Vn, AngEuler, Acc_disturb, self.LAD, self.SPDCMD)
                    
                self.PFAtt2GuidPublisher()
                self.PFAtt2ControlPublisher()

                print("Target Thrust = ", str(self.TargetThrust))
                print("self.TargetAttitude = ", str( self.TargetAttitude))
                
        else : 
            pass
                
    def Waypoint_index_callback(self, msg):
        self.PlannedIndex = msg.waypoint_index
    
    def PFAtt2GuidPublisher(self):
        msg = PFAtt2PFGuid()
        msg.out_ndo = self.outNDO
        self.PFAttToPFGuidPublisher_.publish(msg)
        
    
    def PFAtt2ControlPublisher(self):
        msg = PFAtt2Control()
        msg.timestamp = self.response_timestamp
        msg.target_attitude = self.TargetAttitude
        msg.target_position = self.TargetPosition
        msg.target_thrust = self.TargetThrust
        msg.target_yaw = self.TargetYaw
        self.PFAttToControlPublisher_.publish(msg)
    
    def PF_Guid2PF_AttSubscribe(self, msg):
        self.LAD = msg.lad
        self.SPDCMD = msg.spd_cmd
        
    def PF_ATT_SRV_Callback(self, request, response):
        # print("===== Request Path Following Attitude Command Node =====")
        '''
        ####    SRV INPUT
        uint64 request_timestamp
        bool request_pathfollowing
        float64[] waypoint_x
        float64[] waypoint_y
        float64[] waypoint_z
        ####    MSG INPUT
        uint32 waypoint_index
        float64 lad
        float64 spd_cmd
        uint32 waypoint_index
        '''
        self.requestFlag = request.request_pathfollowing
        self.requestTimestamp = request.request_timestamp
        self.PlannedX = request.waypoint_x
        self.PlannedY = request.waypoint_y
        self.PlannedZ = request.waypoint_z
        if self.requestFlag is True :
            self.PF_ATT_TIMER = self.create_timer(self.PF_ATT_Period, self.PFAttitudeCmdCallback)
            '''
            ####    SRV OUTPUT
            uint64 response_timestamp	# time since system start (microseconds)
            bool response_pathfollowing
            ####    MSG OUTPUT
            float64 targetthrust
            float64[] targetattitude
            float64[] targetposition
            float64 targetyaw
            float64[] outndo
            '''
            response.response_timestamp = self.response_timestamp
            response.response_pathfollowing = True
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_pathfollowing = False
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
        self.phi, self.theta, self.psi = self.Quaternion2Euler(msg.states[0], msg.states[1], msg.states[2], msg.states[3])
        #   Wind Velocity NE
        self.wn = msg.states[22]
        self.we = msg.states[23]
        
        self.Pos = [self.x, self.y, self.z]
        self.Vn = [self.vx, self.vy, self.vz]
        self.AngEuler = [self.phi, self.theta, self.psi]
    
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
    

def main(args=None):
    rclpy.init(args=args)
    
    pf_attitude_cmd_module = PFAttitudeCmdModule()


    try : 
        rclpy.spin(pf_attitude_cmd_module)
    except Exception as e:
                    pf_attitude_cmd_module.get_logger().info(
                        'MPPI module Start failed %r' % (e,))
    finally :
        pf_attitude_cmd_module.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
