import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition

from .submodules.data_buffer import data
from .submodules.functions import q2e

import math
import numpy as np

class AutoPilot(Node):
    def __init__(self):
        super().__init__('auto_pilot')
        #Subscriber
        self.vehicle_attitude_subscriber_ = self.create_subscription(VehicleAttitude,'vehicle_attitude',
        self.vehicle_attitude_callback,10)
        self.vehicle_local_position_subscriber_ = self.create_subscription(VehicleLocalPosition,'vehicle_local_position',
        self.vehicle_local_position_callback,10)
        self.switch_to_autopilot_subscriber_ = self.create_subscription(VehicleLocalPosition,'switch_to_autopilot',
        self.switch_to_autopilot_callback,10)

        #Class Initialization
        self.x=float(0)
        self.y=float(0)
        self.z=float(0)
        self.receive_flag = 0
        self.q = [0] * 4

        #Publisher
        self.autopilot_to_offboardcontrol_publisher_ = self.create_publisher(VehicleLocalPosition,'autopilot_to_offboardcontrol',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.autopilot_to_offboardcontrol_callback)

    def vehicle_attitude_callback(self, msg):
        self.q[0] = msg.q[0]
        self.q[1] = msg.q[1]
        self.q[2] = msg.q[2]
        self.q[3] = msg.q[3]
        # Transform quaternion to euler[rad]
        data.phi, data.theta, data.psi = q2e(self.q[0],self.q[1],self.q[2],self.q[3])
        #print("Euler[phi,theta,psi]=[{} {} {}]".format(data.phi, data.theta, data.psi))
        
    def vehicle_local_position_callback(self, msg):
        local_x = msg.x
        local_y = msg.y
        local_z = msg.z
        #print local position(NED)[meters]
        print("LocalPosition[xyz]=[{} {} {}]".format(local_x,local_y,local_z))

    def switch_to_autopilot_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        # Print local position(NED)[meters]
        print("[Target WPT]\nReceived WPT[xyz]=[{} {} {}]".format(self.x,self.y,self.z))
        self.receive_flag = 1

    def autopilot_to_offboardcontrol_callback(self):
        if (self.receive_flag != 0):
            WPT = VehicleLocalPosition()
            WPT.x = self.x
            WPT.y = self.y
            WPT.z = self.z
            self. autopilot_to_offboardcontrol_publisher_.publish(WPT)
            print("Sended to offboard WPT[xyz]=[{} {} {}]".format(self.x,self.y,self.z))
        else:
            pass

def main(args=None):
        rclpy.init(args=args)

        auto_pilot = AutoPilot()

        rclpy.spin(auto_pilot)

        auto_pilot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 


