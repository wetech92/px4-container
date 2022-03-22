import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition

from .submodules.data_buffer import data
from .submodules.functions import q2e

import math
import numpy as np

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        #Subscriber
        self.vehicle_attitude_subscriber_ = self.create_subscription(VehicleAttitude,'vehicle_attitude',
        self.vehicle_attitude_callback,10)
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleLocalPosition,'vehicle_local_position',
        self.vehicle_local_position_callback,10)
        self.q = [0] * 4
        #Publisher
#       self.vehicle_local_position__publisher_ = self.create_publisher(VehicleLocalPosition,'pathplanning_to_switch',10)
#        timer_period = 0.1
#        self.timer = self.create_timer(timer_period, self.pathplanning_to_switch_callback)

    def vehicle_attitude_callback(self, msg):
        self.q[0] = msg.q[0]
        self.q[1] = msg.q[1]
        self.q[2] = msg.q[2]
        self.q[3] = msg.q[3]
        # Transform quaternion to euler[rad]
        data.phi, data.theta, data.psi = q2e(self.q[0],self.q[1],self.q[2],self.q[3])
        print("Euler[phi,theta,psi]=[{} {} {}]".format(data.phi, data.theta, data.psi))
        #print("{}{}{}{}".format(self.q[0],self.q[1],self.q[2],self.q[3]))
        

    def vehicle_local_position_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        # Print local position(NED)[meters]
        print("LocalPosition[xyz]=[{} {} {}]".format(self.x,self.y,self.z))

#    def pathplanning_to_switch_callback(self)
#       self. 




def main(args=None):
        rclpy.init(args=args)

        collision_avoidance = CollisionAvoidance()

        rclpy.spin(collision_avoidance)

        collision_avoidance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 


