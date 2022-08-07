#! /bin/bash

micrortps_agent -t UDP &
ros2 launch airsim_ros_pkgs airsim_node.launch.py