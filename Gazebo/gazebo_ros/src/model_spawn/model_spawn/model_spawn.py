from cv2 import CV_8UC1, CV_8UC3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
# from model_spawn_srvs.srv import MakeWorld
from model_spawn_srvs.srv import MapGeneration
from sensor_msgs.msg import Image
import time
import os
import random
import cv2
import sys

import numpy as np


class ModelSpawnClass(Node):

    def __init__(self):
        super().__init__('ModelSpawn')
        # Init Grid Map
        self.MapWidth = int(5000)
        self.MapHeight = int(5000)
        self.GridMap = np.full((self.MapWidth, self.MapHeight,3), 255, np.uint8)

        # init Publisher
        self.FireSpawnPublihsher = self.create_publisher(Image, 'MakeFire', 20)
        self.FireTimer = self.create_timer(25, self.FireSpawn)

        self.SpawnEntityClient = self.create_client(SpawnEntity, "/spawn_entity")
        self.SpawnEntityRequest = SpawnEntity.Request()
    
        self.MapGenerationService_ = self.create_service(MapGeneration, 'map_generation', self.MapGenerationCallback)
        
        self.KnownObsSDF = [""] * 3
        self.UnknownObsSDF = [""] * 2

        self.KnownBigObsSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","pine_tree_big","model.sdf")

        self.KnownMiddleObsSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","pine_tree_middle","model.sdf")

        self.KnownSmallObsSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","pine_tree_small","model.sdf")
        self.KnownObsSDF[0] = open(self.KnownSmallObsSDFPath, 'r').read()
        self.KnownObsSDF[1] = open(self.KnownMiddleObsSDFPath, 'r').read()
        self.KnownObsSDF[2] = open(self.KnownBigObsSDFPath, 'r').read()

        self.UnknownBigObsSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","oak_tree_big","model.sdf")
        self.UnknownSmallObsSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","oak_tree_small","model.sdf")
        self.UnknownObsSDF[1] = open(self.UnknownBigObsSDFPath, 'r').read()
        self.UnknownObsSDF[0] = open(self.UnknownSmallObsSDFPath, 'r').read()

        self.GoalSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","oak_tree","model.sdf")
        self.GoalSDF = open(self.GoalSDFPath, 'r').read()

        self.FireSDFPath = os.path.join("/root","PX4-Autopilot","Tools","sitl_gazebo","models","fire","model.sdf")
        self.FireSDF = open(self.FireSDFPath, 'r').read()

        self.ObsPosX = 0.0
        self.ObsPosY = 0.0

        self.MaxBound = 4950.0
        self.MinBound = 50.0

        self.GoalPosX = 5000
        self.GoalPosY = 5000

        self.KnownObsNum = int(sys.argv[1])

        self.KnownObsName = [""] * self.KnownObsNum
        self.KnownObsNamespace = [""] * self.KnownObsNum
        self.KnownObsPosX = [0] * self.KnownObsNum
        self.KnownObsPosY = [0] * self.KnownObsNum
        self.KnownObsIndex = [0] * self.KnownObsNum

        self.UnknownObsNum = int(sys.argv[2])

        self.UnknownObsName = [""] * self.UnknownObsNum
        self.UnknownObsNamespace = [""] * self.UnknownObsNum
        self.UnknownPosX = [0.0] * self.UnknownObsNum
        self.UnknownPosY = [0.0] * self.UnknownObsNum

        self.WindVelX = 2.0
        self.WindVelY = -1.0

        self.FireIndex = 0

        self.FirePosX = random.uniform(400, 500)
        self.FirePosY = random.uniform(100, 300)
        self.FireName = [""] * self.KnownObsNum
        self.FireNamespace = [""] * self.KnownObsNum

        self.MakeWorldDone = 0
        self.InitFlag = True

        self.FireObsName = ""
        self.FireObsNamespace = ""
        
        self.worldGenerationRequset = False
        self.mapGeneration = False
        self.get_logger().info('running...')

        
    
    def MapGenerationCallback(self, request, response):
        self.get_logger().info("===== Request Map Generation =====")
        self.worldGenerationRequset = request.world_generation_request
        if self.worldGenerationRequset == True :
            self.MakeWorld()
            response.map_generation = self.mapGeneration
            print("=== Map Generation ===")
            return response
        else :
            self.mapGeneration = False
        
    # Goal Spawn
    def MakeWorld(self):
        #self.GoalSpawn()
        self.UnknownObsSpawn()
        self.KnownObsSpawn()
        time.sleep(10)
        # self.RequestMakeWorldDone()
        self.mapGeneration = True

    # Goal Spawn
    def GoalSpawn(self):
        self.SendRequestSpawnEntity("Goal", "Goal", self.GoalSDF, self.GoalPosX, self.GoalPosY)

    # Fire Spawn
    def FireSpawn(self):
        if self.MakeWorldDone == 1:
            print("Fire Spreading")
            self.FireIndex += 1
            self.FirePosX += 10
            self.FirePosY += 10
            self.FireName = "Fire" + str(self.FireIndex)
            self.FireNamespace = "Fire" + str(self.FireIndex)
            self.SendRequestSpawnEntity(self.FireName, self.FireNamespace, self.FireSDF, self.FirePosX, self.FirePosY)

    # In Obstacle Spawn
    def UnknownObsSpawn(self):
        for i in range(0, self.UnknownObsNum - 1):
            index = random.randint(1,2) - 1
            self.UnknownPosX[i] = random.randint(self.MinBound / 10, self.MaxBound / 10) 
            self.UnknownPosY[i] = random.randint(self.MinBound / 10, self.MaxBound / 10) 
            self.UnknownObsName[i] = "UnknownObs" + str(i)
            self.UnknownObsNamespace[i] = "UnknownObs" + str(i)
            self.SendRequestSpawnEntity(self.UnknownObsName[i], self.UnknownObsNamespace[i], self.UnknownObsSDF[index], self.UnknownPosX[i], self.UnknownPosY[i])

    # Out Obstacle Spawn
    def KnownObsSpawn(self):
        Color = (0, 0, 0)
        for i in range(0, self.KnownObsNum - 1):
            index = random.randint(1,3) - 1
            self.KnownObsIndex[i] = index + 1
            self.KnownObsPosX[i] = random.randint(self.MinBound / 10, self.MaxBound / 10) 
            self.KnownObsPosY[i] = random.randint(self.MinBound / 10, self.MaxBound / 10) 
            self.KnownObsName[i] = "KnownObs" + str(i)
            self.KnownObsNamespace[i] = "KnownObs" + str(i)
            self.SendRequestSpawnEntity(self.KnownObsName[i], self.KnownObsNamespace[i], self.KnownObsSDF[index], self.KnownObsPosX[i], self.KnownObsPosY[i])
            #Center = (self.KnownObsPosX[i] * 10 , self.KnownObsPosY[i] * 10)
            Center = (int(self.KnownObsPosX[i] * 10) , int(self.KnownObsPosY[i] * 10))
            if index == 0:
                Temp = 4
            elif index == 1:
                Temp = 8
            elif index == 2:
                Temp = 10
            #Radius = int(Temp / 2)
            Radius = int(Temp / 2 * 20)
            
            cv2.circle(self.GridMap, Center, Radius, Color, -1)

        cv2.rotate(self.GridMap, cv2.ROTATE_180)
        cv2.flip(self.GridMap,1)

        cv2.cvtColor(self.GridMap, cv2.COLOR_RGB2GRAY)
        cv2.threshold(self.GridMap, 120, 255, cv2.THRESH_BINARY)
        cv2.imwrite("/root/ros_ws/src/a4vai/a4vai/path_planning/Map/RawImage.png",self.GridMap)


        self.get_logger().info("===== Map Save!! =====")

    ## Client
    # SpawnEntity
    def SendRequestSpawnEntity(self, ModelName, ModelNamespace, ModelPath, ModelPosX, ModelPosY):

        self.SpawnEntityRequest.name = ModelName
        self.SpawnEntityRequest.xml = ModelPath
        self.SpawnEntityRequest.robot_namespace = ModelNamespace

        self.SpawnEntityRequest.initial_pose.position.x = float(ModelPosX)
        self.SpawnEntityRequest.initial_pose.position.y = float(ModelPosY)
        self.SpawnEntityRequest.initial_pose.position.z = 0.0
        #self.SpawnEntityRequest.initial_pose.orientation.w = 1.0
        #self.SpawnEntityRequest.initial_pose.orientation.x = 0.0
        #self.SpawnEntityRequest.initial_pose.orientation.y = 0.0
        #self.SpawnEntityRequest.initial_pose.orientation.z = 0.0
        #self.get_logger().info("Sending service request to `/spawn_entity`")
        future = self.SpawnEntityClient.call_async(self.SpawnEntityRequest)
        if future.result() is not None:
            print('response: %r' % future.result())



def main(args=None):
    rclpy.init(args=args)

    ModelSpawn = ModelSpawnClass()

    rclpy.spin(ModelSpawn)
    ModelSpawn.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

