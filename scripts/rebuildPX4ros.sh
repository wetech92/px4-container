#! /bin/bash

# Update urtps_bridge_topics.yaml from PX4-Autopilot/msg

rm -rf /root/px4_ros/src/px4_msgs/msg/*

python3 /root/PX4-Autopilot/msg/tools/uorb_to_ros_urtps_topics.py \
    -i /root/PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml \
    -o /root/px4_ros/src/px4_ros_com/templates/urtps_bridge_topics.yaml

python3 /root/PX4-Autopilot/msg/tools/uorb_to_ros_msgs.py \
    /root/PX4-Autopilot/msg/ \
    /root/px4_ros/src/px4_msgs/msg/

colcon build --build-base ~/px4_ros/build \
    --install-base ~/px4_ros/install \
    --base-paths ~/px4_ros/src \
    --symlink-install \
    --event-handlers console_direct+