from requests import request
import numpy as np
import math

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
from px4_msgs.msg import EstimatorStates

from msg_srv_act_interface.srv import CollisionAvoidanceSetpoint

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from .losca import CollisionAvoidance_jy
from .JBNU_Obs import JBNU_Collision

# Opencv-ROS
import cv2


class Collision_Avoidance(Node):
    def __init__(self):
        super().__init__('losca_module')
        print(" losca Module In ")
        ## Input
        self.x = 0.0
        self.y = 0.0
        self.ObsAngle = 0.0
        self.ObsSize  = 0.0
        # self.JYCollision = CollisionAvoidance_jy()
        self.JBNUCollision = JBNU_Collision()
        self.current_frame = []
        ##  Output
        self.vel_cmd_x = 0.0
        self.vel_cmd_y = 0.0
        self.vel_cmd_z = 0.0
        self.vel_cmd_yaw = 0.0
        self.qosProfileGen()
        self.requestFlag = False
        self.response_timestamp = 0
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, self.QOS_Sub_Sensor)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, self.QOS_Sub_Sensor)
        self.LidarSubscriber_ = self.create_subscription(LaserScan, '/rplidar_a3/laserscan', self.LidarCallback, self.QOS_Sub_Sensor)
        self.CollisionAvoidanceService_ = self.create_service(CollisionAvoidanceSetpoint, 'collision_avoidance', self.CollisionAvoidanceCallback)
        self.CameraSubscriber_ = self.create_subscription(Image, '/realsense_d455_depth/realsense_d455_depth/depth/image_raw', self.CameraCallback, self.QOS_Sub_Sensor)


        
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
        
    def CollisionAvoidanceCallback(self, request, response):
        print("===== Request Coliision Avoidance Node =====")
        self.requestTimestamp = request.request_timestamp
        if request.request_collisionavoidance is True : 
            print(type(self.requestFlag))
            # self.vel_cmd_x, self.vel_cmd_y, self.vel_cmd_z, self.vel_cmd_yaw = self.JYCollision.CA(self.x, self.y, self.ObsAngle, self.ObsSize, self.requestFlag)
            self.vel_cmd_x, self.vel_cmd_y, self.vel_cmd_z, self.vel_cmd_yaw = self.JBNUCollision.CA(self.current_frame)
            print("===== Coliision Avoidance Complete!! =====")
            response.response_timestamp = self.response_timestamp
            response.response_collisionavoidance = True
            response.vel_cmd_x = self.vel_cmd_x
            response.vel_cmd_y = self.vel_cmd_y
            response.vel_cmd_z = self.vel_cmd_z
            response.vel_cmd_yaw = self.vel_cmd_yaw
            print("===== Response Coliision Avoidance Node =====")
            return response
        else : 
            response.response_timestamp = self.response_timestamp
            response.response_collisionavoidance = False
            response.vel_cmd_x = 0.0
            response.vel_cmd_y = 0.0
            response.vel_cmd_z = 0.0
            response.vel_cmd_yaw = 0.0
            return response
        
    def TimesyncCallback(self, msg):
        self.response_timestamp = msg.timestamp

    # EstimatorStates
    def EstimatorStatesCallback(self, msg):
        # TimeStamp
        self.EstimatorStatesTime = msg.timestamp
        # Position NED
        self.x = msg.states[7]
        self.y = msg.states[8]
        
        return self.x, self.y
        '''
        self.z = msg.states[9]

            # Velocity NED
        self.vx = msg.states[4]
        self.vy = msg.states[5]
        self.vz = msg.states[6]

        # Attitude
        self.roll, self.pitch, self.yaw = self.Quaternion2Euler(msg.states[0], msg.states[1], msg.states[2], msg.states[3])
        # Wind Velocity NE
        self.wn = msg.states[22]
        self.we = msg.states[23]
        '''
        
    def LidarCallback(self, msg):
        self.ObsDist = min(msg.ranges)
        ObsDist2 = max(msg.ranges)
        self.ObsPos[0] = self.ObsDist * math.cos(self.ObsAngle * math.pi / 180)
        self.ObsPos[1] = self.ObsDist * math.sin(self.ObsAngle * math.pi / 180)
        self.ObsAngle = 3.6 * math.argmin(msg.ranges)
        self.ObsSizeAngle = (3.6 * (100 - msg.ranges.count(math.inf))) / 2
        self.ObsSize = 2 * (self.ObsDist * math.tan(self.ObsSizeAngle * self.R2D))
        self.requestFlag = False
        # print(" min Distance : %f"%self.ObsDist)
        # print("Angle : %f"%self.ObsAngle)
        # print("max Distance : %f"%self.ObsDist2)
        #print("Position X : %f"%self.ObsPos[0])
        #print("Position Y : %f"%self.ObsPos[1])
        #print("X : %f"%self.ObsPos[0],"Y : %f"%self.ObsPos[1])
        # print(" ObsSize : %f"%self.ObsSize )
        # print(" ObsSizeAngle : %f"%self.ObsSizeAngle )
        
        if self.ObsDist < 4.0:
            self.requestFlag = True
            
    def CameraCallback(self, msg):
        self.current_frame = self.CvBridge.imgmsg_to_cv2(msg)
        cv2.imshow("depth", self.current_frame)
        cv2.waitkey(0.1)