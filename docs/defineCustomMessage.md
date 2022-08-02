# Defining Custom UORB/uRTPS Messages

> This document is based on **Release v.1.13.0 of PX4-Autopilot**
> For Different release of PX4-Autopilot, content of this document may be invalid

## Files & Scripts Used

|NAME|DIRECTORY|TYPE|
|:-|:-|:-|
|`<MESSAGE_NAME>.msg`|`PX4-Autopilot/msg/`|ADD|
|`CMakeLists.txt`|`PX4-Autopilot/msg/`|MODIFY|
|`urtps_bridge_topics.yaml`|`PX4-Autopilot/msg/tools/`|MODIFY|
|`uorb_to_ros_urtps_topics.py`|`/PX4-Autopilot/msg/tools/`|USE|
|`uorb_to_ros_msgs.py`|`/PX4-Autopilot/msg/tools/`|USE|

## Overview

![PX4_ROS2_Bridge](images/PX4_ROS2_Bridge.png) [1]

- PX4 uses uORB (Micro Object Request Broker) message to handle communications betweem internal threads/processes
- On the other hand, ROS2 uses DDS (Data Distribution Service) for comuunications between ROS2 nodes/services
- Therefore, in order to control PX4 from ROS2, ORB message must be transformed into DDS message
- RTPS (Real Time Publish Subscribe) layer plays a role in this part<sub>[2]</sub>. Therefore, we should set list of messages to be published/subscribed.

## Defining custom uORB message

- Check whether there's a message you want to publish/subscribe is defined or not
  - uORB messages are named in the snake case and has a filename extension of `.msg`
  - If there is already a message you want, skip this part and head over to the next part.

```shell
cd ~/PX4-Autopilot/msg
ls | grep *.msg
```

- If there is no message you want, you have to define it.
- Define new `.msg` file based on your need. Checking other `.msg` files will help
  - Keep in mind that all messages must include `uint64 timestamp` for logging purpose
- Example: `vehicle_attitude_setpoint`

```cpp
uint64 timestamp		# time since system start (microseconds)

float32 roll_body		# body angle in NED frame (can be NaN for FW)
float32 pitch_body		# body angle in NED frame (can be NaN for FW)
float32 yaw_body		# body angle in NED frame (can be NaN for FW)

float32 yaw_sp_move_rate	# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d			# Desired quaternion for quaternion control

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body		# Normalized thrust command in body NED frame [-1,1]

bool roll_reset_integral	# Reset roll integral part (navigation logic change)
bool pitch_reset_integral	# Reset pitch integral part (navigation logic change)
bool yaw_reset_integral	# Reset yaw integral part (navigation logic change)

bool fw_control_yaw		# control heading with rudder (used for auto takeoff on runway)

uint8 apply_flaps       	# flap config specifier
uint8 FLAPS_OFF = 0     	# no flaps
uint8 FLAPS_LAND = 1    	# landing config flaps
uint8 FLAPS_TAKEOFF = 2 	# take-off config flaps

uint8 apply_spoilers		# spoiler config specifier
uint8 SPOILERS_OFF = 0     	# no spoilers
uint8 SPOILERS_LAND = 1    	# landing config spoiler
uint8 SPOILERS_DESCEND = 2 	# descend config spoiler

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
```

- After defining new message, you must also modify `CMakeLists.txt` to make that message to be built.
- Add your message under `set(msg_files ...` in `PX4-Autopilot/msg/CMakeLists.txt` 

## Specifying uORB message in publish/subscription list

- Publish and subscription of uORB message is controlled by `urtps_bridge_topics.yaml`
- Messages are listed in following form:

```yaml
rtps:
  - msg:     debug_array
    receive: true
  - msg:     debug_key_value
    receive: true
  - msg:     debug_value
    receive: true
...
```

- Statements of this `.yaml` file are as follow:
  - `msg:` defines a target message to be set. Target must be a message defined in `PX4-Autopilot/msg`
  - Then, statements `send:true`, or `receive:true` follows to publish/subscribe a message
    - This is written based on client's point of view. `receive:true` means that PX4 receives a message
  - Using `base:` statement, you can set an 'alias' of a message
- As shown, not all messages are stated. Therefore, you cen see that only stated messages can be interacted.

## Specifying uRTPS message in publish/subscription list

- Next, uRTPS message and uRTPs topic list must be redefiend based on modifications made on PX4-Autopilot
- [The official document](https://docs.px4.io/v1.12/en/ros/ros2_comm.html) does this by cloning [`PX4/px4_msgs`](https://github.com/PX4/px4_msgs/blob/master/msg/ActuatorTest.msg) and [`PX4/px4_ros_com`](https://github.com/PX4/px4_ros_com)
- However, these repositories only include message settings of default releases of PX4-Autopilot
- Therefore, after modifiying message settings in PX4-Autopilot, additional modifications should also be made on the agent
<br/>

- All msgs named in the camel case must be renamed in the snake case. Renaming is a very repeative, time-consuming task.
- However, PX4-Autopilot already provides convenience python scripts for these tasks in `msg/tools`
  - Message renaming: `uorb_to_ros_msgs.py` / List renaming: `uorb_to_ros_urtps_topics.py`
<br/>

- First, clear-up messages in `px4_msgs` and fill with newly renamed messages using uorb_to_ros_msgs.py

```shell
rm -rf ~/px4_ros/src/px4_msgs/msg/*
python3 ~/PX4-Autopilot/msg/tools/uorb_to_ros_msgs.py \
        ~/PX4-Autopilot/msg/ \
        ~/px4_ros/src/px4_msgs/msg/
```

- Then, do the similar task on `urtps_bridge_topics.yaml`.

```shell
rm -rf ~/px4_ros/src/px4_ros_com/templates urtps_bridge_topics.yaml
python3 ~/PX4-Autopilot/msg/tools/uorb_to_ros_urtps_topics.py \
        -i ~/PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml \
        -o ~/px4_ros/src/px4_ros_com/templates/urtps_bridge_topics.yaml
```

- Finally, rebuild `px4_msgs` and `px4_ros_com` ROS2 packages and install them

```shell
colcon build --build-base ~/px4_ros/build \
        --install-base ~/px4_ros/install \
        --base-paths ~/px4_ros/src \
        --symlink-install \
        --event-handlers console_direct+
...
source ~/px4_ros/install/setup.bash
```

## Reference

1. [PX4 User Guide: ROS 2 User Guide (PX4-ROS 2 Bridge)](https://docs.px4.io/v1.12/en/ros/ros2_comm.html)
2. [FastDDS: 2.Library Overview - 2.1. Architecture](https://fast-dds.docs.eprosima.com/en/latest/fastdds/library_overview/library_overview.html#architecture)