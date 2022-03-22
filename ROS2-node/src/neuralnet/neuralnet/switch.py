import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

from .submodules.data_buffer import data
from .submodules.functions import q2e

import math
import numpy as np

class Switch(Node):
    def __init__(self):
        super().__init__('switch')
        #Subscriber
        self.pathplanning_to_switch_subscriber_ = self.create_subscription(VehicleLocalPosition,'pathplanning_to_switch',
        self.pathplanning_to_switch_callback,10)
        self.pathplanning_to_switch_subscriber_
        #Publisher
        self.switch_to_autopilot_publisher_ = self.create_publisher(VehicleLocalPosition,'switch_to_autopilot',10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.switch_to_autopilot_callback)
        #Class Initialization
        self.x=float(0)
        self.y=float(0)
        self.z=float(0)
        self.receive_flag = 0
        

    def pathplanning_to_switch_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        # Print local position(NED)[meters]
        print("[Path Planning mode]\nReceived WPT[xyz]=[{} {} {}]".format(self.x,self.y,self.z))
        self.receive_flag = 1

    def switch_to_autopilot_callback(self):
        if (self.receive_flag != 0):
            WPT = VehicleLocalPosition()
            WPT.x = self.x
            WPT.y = self.y
            WPT.z = self.z
            self. switch_to_autopilot_publisher_.publish(WPT)
            print("Sended to Autopilot WPT[xyz]=[{} {} {}]".format(self.x,self.y,self.z))
        else:
            pass


def main(args=None):
        rclpy.init(args=args)

        switch = Switch()

        rclpy.spin(switch)

        switch.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 


