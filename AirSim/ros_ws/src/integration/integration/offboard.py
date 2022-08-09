import rclpy
from rclpy.node import Node

# PX4 MSG Subscriber
from px4_msgs.msg import EstimatorStates
from px4_msgs.msg import VehicleAngularVelocity

# PX4 MSG Publisher
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleRatesSetpoint

# Camera Subscriber
from sensor_msgs.msg import Image

# Lidar Subscriber
from sensor_msgs.msg import LaserScan

# Opencv-ROS
from cv_bridge import CvBridge
import cv2

# Time
import time

# Matplotlib
import matplotlib.pyplot as plt

# Numpy
import numpy as np
import matplotlib.pyplot as plt

## Client
# Reset, Pause, Unpause  SRV
# from std_srvs.srv import Empty

# MakeWorld SRV
# from model_spawn_srvs.srv import MakeWorld

# Math
import math

## Collision Avoidance Module
#  Artificial Potential Field
# from .CollisionAvoidance.ArtificialPotentialField import ArtificialPotentialField

## Path Planning Module
#  RRT
from .PathPlanning.RRT import RRT

import time


class IntegrationNode(Node):

    def __init__(self):
        super().__init__('integration')
        # Init PathPlanning Module
        self.RRT = RRT.RRT()

        # Init CVBridge
        self.CvBridge = CvBridge()

        # Init Aritificial Potential Field
        # self.APF = ArtificialPotentialField.ArtificialPotentialField(10, 10, 10, 10)

        # init PX4 MSG Publisher
        self.VehicleCommandPublisher_ = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', 10)
        self.OffboardControlModePublisher_ = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', 10)
        self.TrajectorySetpointPublisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.VehicleAttitudeSetpointPublisher_ = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/vehicle_attitude_setpoint/in', 10)
        self.VehicleRatesSetpointPublisher_ = self.create_publisher(VehicleRatesSetpoint, '/fmu/vehicle_rates_setpoint/in', 10)

        # init PX4 MSG Subscriber
        self.TimesyncSubscriber_ = self.create_subscription(Timesync, '/fmu/time_sync/out', self.TimesyncCallback, 10)
        self.EstimatorStatesSubscriber_ = self.create_subscription(EstimatorStates, '/fmu/estimator_states/out', self.EstimatorStatesCallback, 10)
        self.VehicleAngularVelocitySubscriber_ = self.create_subscription(VehicleAngularVelocity, '/fmu/vehicle_angular_velocity/out', self.VehicleAngularVelocityCallback, 10)

        # Init Camera Subscriber
        self.CameraSubscriber_ = self.create_subscription(Image, '/airsim_node/Typhoon_1/OptCamera/Scene', self.CameraCallback, 60)

        # Init Lidar Subscriber
        self.LidarSubscriber_ = self.create_subscription(LaserScan, '/airsim_node/Typhoon_1/lidar/RPLIDAR_A3', self.LidarCallback, 10)


        # # Init Client
        # self.ResetWorldClient = self.create_client(Empty, '/reset_world')
        # self.ResetWorldClientRequest = Empty.Request()
        # #while not self.ResetWorldClient.wait_for_service(timeout_sec=1.0):
        # #    self.get_logger().info('service not available, waiting again...')
        
        # self.PauseClient = self.create_client(Empty, '/pause_physics')
        # self.PauseClientRequest = Empty.Request()
        
        # self.UnpauseClient = self.create_client(Empty, '/unpause_physics')
        # self.UnpauseClientRequest = Empty.Request()
        
        # self.MakeWorldService = self.create_service(MakeWorld, 'make_world', self.MakeWorldCallback)
        

        # Offboard Period
        OffboardPeriod = 0.01
        self.OffboardCounter = 1 / OffboardPeriod
        self.OffboardTimer = self.create_timer(OffboardPeriod, self.OffboardControl)

        # Timestamp
        self.timestamp = 0.0

        self.timestamp2 = 0

        # Offboard Mode Counter
        self.OffboardCount = 0

        # Arm Disarm Command
        self.VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

        # Offboard Mode Command
        self.VEHICLE_CMD_DO_SET_MODE = 176

        # Vehicle States Variables
        self.x = 10.0
        self.y = 10.0
        self.z = 10.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        ## Controller Sample Variables
        self.TargetPosition = [0.5, 0.5, -10.0] # Meter
        self.TargetVelocity = [0.5, 0.5, 0.0] # Meter
        self.TargetAttitude = [0.7071, 0, 0, 0.7071] # Quaternion w x y z
        self.TargetRate = [0.5, 0.0, 0.1] # Radian
        self.TargetThrust = 0.33
        self.TargetBodyRate = [np.NaN, np.NaN, np.NaN]
        self.TargetYawRate = 0.0


        # ## Collision Avoidance Variables
        # self.CollisionAvoidanceFlag = False
        # self.LidarSampling = 0
        # self.AvoidancePos = [0.0] * 2
        # self.CA = [0.0] * 2
        # self.ObsDist = 0.0

        ## Path Planning Variables
        self.Target = [ 480.0, 480.0, -5.0]

        # TakeOff Variables
        self.InitialPosition = [ 0.0, 0.0, -5.0]
        self.InitialPositionFlag = False

        self.StartPoint = np.array([[100], [100]])
        self.GoalPoint = np.array([[4900], [4900]])
        
        self.RawImage = (cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png", cv2.IMREAD_GRAYSCALE))
        self.Image = np.uint8(np.uint8((255 - self.RawImage)/ 255))
        self.Image = cv2.flip(self.Image, 0)
        # self.Image = cv2.rotate(self.Image, cv2.ROTATE_90_CLOCKWISE)
        self.Planned = self.RRT.PathPlanning(self.Image, self.StartPoint, self.GoalPoint)
        self.PlannedX = (self.Planned[0] / 10)
        self.PlannedY = (self.Planned[1] / 10)
        self.MaxPlannnedIndex = len(self.PlannedX) - 1
        print(len(self.PlannedX))
        self.PathPlanningInitialize = True

        # Plot Generated Path on Plot
        self.bgd = plt.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png")
        plt.imshow(self.bgd,zorder=0, extent=[0, 5000, 0, 5000])
        plt.plot(self.PlannedX*10,self.PlannedY*10)
        plt.savefig('/root/ros_ws/src/integration/integration/PathPlanning/Map/Path.png')

        self.PlannedX = self.PlannedX - 10
        self.PlannedY = self.PlannedY - 10

        self.LogFile = open("/root/ros_ws/src/integration/integration/PathPlanning/Map/log.txt",'a')
        self.PlannnedIndex = 0
        
        self.PathPlanningTargetPosition = np.array([0.0, 0.0, 0.0])

        
    ######################################################################################################################################## 
    # Main Function
    def OffboardControl(self):
        if self.PathPlanningInitialize == True:
            if self.OffboardCount == self.OffboardCounter:
                self.offboard()
                self.arm()
            
            self.OffboardControlModeCallback()

            if self.InitialPositionFlag:
                ###########################################
                # Sample PathPlanning Example
                
                if self.PlannnedIndex >= self.MaxPlannnedIndex:
                    self.LogFile.close()
                    print("DONE")
                else:
                    self.PathPlanningTargetPosition = np.array([self.PlannedX[self.PlannnedIndex], self.PlannedY[self.PlannnedIndex], -5.0])
                    self.TargetYaw = np.arctan2(self.Target[1] - self.y, self.Target[0] - self.x)
                    if self.PlannnedIndex > 400:
                        self.TargetYaw = np.arctan2(self.Target[1] - 0, self.Target[0] - 0)
                    self.SetPosition(self.PathPlanningTargetPosition, self.TargetYaw)
                    # self.LogFile = open("/root/ros_ws/src/integration/integration/PathPlanning/Map/log.txt",'a')
                    print(np.array([self.x, self.y]))
                    print(np.array([self.PlannedX[self.PlannnedIndex], self.PlannedY[self.PlannnedIndex]]))
                    WaypointACK = np.linalg.norm(np.array([self.PlannedX[self.PlannnedIndex], self.PlannedY[self.PlannnedIndex]]) - np.array([self.x, self.y]))
                    if  WaypointACK < 5:
                        LogData = "%d %f %f %f %f\n" %(self.PlannnedIndex, self.PlannedX[self.PlannnedIndex], self.PlannedY[self.PlannnedIndex], self.x, self.y)
                        self.LogFile.write(LogData)
                        self.PlannnedIndex += 1
                        # print(self.PlannnedIndex)
                
                ###########################################
                ##########################################
                # Sample Collision Avoidance Example
                """
                if self.CollisionAvoidanceFlag == True:
                    SetPosition = np.array([self.Target[0] -self.CA[0],  self.Target[0] -self.CA[1], -5.0])
                    self.SetPosition(SetPosition)
                else:
                    self.SetPosition(self.Target)
                """
                ###########################################

                ###########################################
                # Sample Control Position, Velocity, Attitude, Rate    
                #self.SetPosition(self.TargetPosition, self.TargetYaw)
                #self.SetVelocity(self.TargetVelocity, self.TargetYaw)
                #self.SetAttitude(self.TargetAttitude, self.TargetBodyRate, self.TargetThrust, self.TargetYawRate)
                #self.SetRate(self.TargetRate, self.TargetThrust)
            
            else:
                self.Takeoff()
                print("Takeoff")
                print(np.array([self.x,self.y]))
                print(np.array([self.PlannedX[0],self.PlannedY[0]]))
                
            if self.OffboardCount < self.OffboardCounter:
                self.OffboardCount = self.OffboardCount + 1
    ########################################################################################################################################
    
    # # MakeWorld
    # def MakeWorldCallback(self, request, response):
    #     if request.done == 1:
    #         print("Requset")
    #         RawImage = (cv2.imread("/root/ros_ws/src/integration/integration/PathPlanning/Map/test.png", cv2.IMREAD_GRAYSCALE))
    #         Image = np.uint8(np.uint8((255 - RawImage)/ 255))
    #         Image = cv2.flip(Image, 0)
    #         Image = cv2.rotate(Image, cv2.ROTATE_90_CLOCKWISE)
            
    #         Planned = self.RRT.PathPlanning(Image, self.StartPoint, self.GoalPoint)
    #         RawImage = cv2.flip(RawImage, 0)
    #         cv2.imwrite('rawimage.png',RawImage)
    #         self.PlannedX = Planned[0] / 10
    #         self.PlannedY = Planned[1] / 10
    #         self.MaxPlannnedIndex = len(self.PlannedX) - 1
    #         print(len(self.PlannedX))
    #         response.ack = 1
    #         self.PathPlanningInitialize = True
    #         return response
            
    ## Vehicle Mode
    # Arming
    def arm(self):
        self.VehicleCommandCallback(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

    # Disarming
    def disarm(self):
        self.VehicleCommandCallback(self.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)

    # Offboard
    def offboard(self):
        self.VehicleCommandCallback(self.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    # ## Gazebo User Level Fucntion
    # # Gazebo Reset 
    # def Reset(self):
    #     self.SendResetWorld()

    # # Gazebo Pause
    # def Pause(self):
    #     self.SendPause()

    # # Gazebo Unpause
    # def Unpause(self):
    #     self.SendUnpause()
        
    ## PX4 User Level Function
    # Takeoff
    def Takeoff(self):
        self.SetPosition(self.InitialPosition, 0.0)
        if abs(self.z - self.InitialPosition[2]) < 0.3:
            self.InitialPositionFlag = True
            

    ## PX4 Controller
    # Set Position
    def SetPosition(self, SetPosition, SetYaw):
        SetVelocity = [np.NaN, np.NaN, np.NaN]
        self.TrajectorySetpointCallback(SetPosition, SetVelocity, SetYaw)
        
    # Set Velocity
    def SetVelocity(self, SetVelocity, SetYaw):
        SetPosition = [np.NaN, np.NaN, np.NaN]
        self.TrajectorySetpointCallback(SetPosition, SetVelocity, SetYaw)

    # Set Attitude
    def SetAttitude(self, SetQuaternion, BodyRate, SetThrust, SetYawRate):
        self.VehicleAttitudeSetpointCallback(SetQuaternion, BodyRate, SetThrust, SetYawRate)
    
    # Set Rate        self.PlannedX = self.PlannedX - 10
        self.PlannedY = self.PlannedY - 10
    ## PX4 Publisher
    # VehicleCommand
    def VehicleCommandCallback(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp2
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        self.VehicleCommandPublisher_.publish(msg)

    # OffboardControlMode
    def OffboardControlModeCallback(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp2
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        self.OffboardControlModePublisher_.publish(msg)

    # TrajectorySetpoint
    def TrajectorySetpointCallback(self, SetPosition, SetVelocity, SetYaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp2
        msg.x = SetPosition[1]
        msg.y = SetPosition[0]
        msg.z = SetPosition[2]
        msg.vx = SetVelocity[0]
        msg.vy = SetVelocity[1]
        msg.vz = SetVelocity[2]
        msg.yaw = SetYaw

        self.TrajectorySetpointPublisher_.publish(msg)
        
            
    # VehicleAttitudeSetpoint
    def VehicleAttitudeSetpointCallback(self, SetQuaternion, BodyRate, SetThrust, SetYawRate):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.timestamp2

        msg.roll_body = BodyRate[0]
        msg.pitch_body = BodyRate[1]
        msg.yaw_body = BodyRate[2]
        
        msg.q_d[0] = SetQuaternion[0]
        msg.q_d[1] = SetQuaternion[1]
        msg.q_d[2] = SetQuaternion[2]
        msg.q_d[3] = SetQuaternion[3]
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -SetThrust
        msg.yaw_sp_move_rate = SetYawRate
        
        self.VehicleAttitudeSetpointPublisher_.publish(msg)
    
    # VehicleRatesSetpoint
    def VehicleRatesSetpointCallback(self, SetRate, SetThrust):
        msg = VehicleRatesSetpoint()
        
        msg.timestamp = self.timestamp2
        msg.roll = SetRate[0]
        msg.pitch = SetRate[1]
        msg.yaw  = SetRate[2]
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -SetThrust
        
        self.VehicleRatesSetpointPublisher_.publish(msg)
        

    ## Subscriber
    # VehicleAngularVelocity
    def VehicleAngularVelocityCallback(self, msg):

        # Rate
        self.p = msg.xyz[0] * 57.2958
        self.q = msg.xyz[1] * 57.2958
        self.r = msg.xyz[2] * 57.2958
        #print(self.p, self.q, self.r)

    # EstimatorStates
    def EstimatorStatesCallback(self, msg):
        
        # TimeStamp
        self.EstimatorStatesTime = msg.timestamp
        
        # Position NED
        self.x = msg.states[8]
        self.y = msg.states[7]
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

    # Timesync
    def TimesyncCallback(self, msg):
        self.timestamp2 = msg.timestamp

    # ## Gazebo Client
    # # Empty
    # def SendResetWorld(self):
    #     self.ResetWorldClient.call_async(self.ResetWorldClientRequest)

    # # Pause
    # def SendPause(self):
    #     self.PauseClient.call_async(self.PauseClientRequest)

    # # Unpause
    # def SendUnpause(self):
    #     self.UnpauseClient.call_async(self.UnpauseClientRequest)

    ## Gazebo Sensor Plugin
    # Camera
    def CameraCallback(self, msg):
        current_frame = self.CvBridge.imgmsg_to_cv2(msg)
        # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

    # Lidar
    def LidarCallback(self, msg):
        ObsPos = [0.0] * 2
        ObsDist = min(msg.ranges)
        print(ObsDist)
        self.CollisionAvoidanceFlag = False
        if ObsDist < 10.0:
            ObsAngle = np.argmin(msg.ranges)
            ObsPos = [ObsDist * math.sin(ObsAngle * math.pi / 180), ObsDist * math.cos(ObsAngle * math.pi / 180)]
            self.CA = self.APF.CalTotalForce([self.Target[0], self.Target[1]], self.AvoidancePos, ObsPos)
            print(ObsAngle)
            self.CollisionAvoidanceFlag = True

    ## Mathmatics Function
    # Quaternion to Euler
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
    
    # Euler to Quaternion
    def Euler2Quaternion(self, Roll, Pitch, Yaw):
        CosYaw = math.cos(Yaw * 0.5)
        SinYaw = math.sin(Yaw * 0.5)
        CosPitch = math.cos(Pitch * 0.5)
        SinPitch = math.sin(Pitch * 0.5)
        CosRoll = math.cos(Roll * 0.5)
        SinRoll= math.sin(Roll * 0.5)
        
        w = CosRoll * CosPitch * CosYaw + SinRoll * SinPitch * SinYaw
        x = SinRoll * CosPitch * CosYaw - CosRoll * SinPitch * SinYaw
        y = CosRoll * SinPitch * CosYaw + SinRoll * CosPitch * SinYaw
        z = CosRoll * CosPitch * SinYaw - SinRoll * CosPitch * CosYaw
        
        return w, x, y, z



def main(args=None):
    rclpy.init(args=args)

    Integration = IntegrationNode()
    rclpy.spin(Integration)
    Integration.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
