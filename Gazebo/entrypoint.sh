#!/bin/bash

# Start virtual X server in the background
# - DISPLAY default is :99, set in dockerfile
# - Users can override with `-e DISPLAY=` in `docker run` command to avoid
#   running Xvfb and attach their screen
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
	echo "Starting Xvfb"
	Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# Check if the ROS_DISTRO is passed and use it
# to source the ROS environment
if [ -n "${ROS_DISTRO}" ]; then
	source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Use the LOCAL_USER_ID if passed in at runtime
if [ -n "${LOCAL_USER_ID}" ]; then
	echo "Starting with UID : $LOCAL_USER_ID"
	# modify existing user's id
	usermod -u $LOCAL_USER_ID user
	# run as user
	exec gosu user "$@"
else
	exec "$@"
fi

# ------------A4VAI DEFINED ENTRYPOINT-------------
echo "    ___   __ __      _    _____    ____"
echo "   /   | / // /     | |  / /   |  /  _/"
echo "  / /| |/ // /______| | / / /| |  / /  "
echo " / ___ /__  __/_____/ |/ / ___ |_/ /   "
echo "/_/  |_| /_/        |___/_/  |_/___/   "

source /opt/ros/galactic/setup.bash

# Rebuild ALL ROS2 nodes if activated
if [[ -n ${REBUILD_RPKG_INTEGRATION} ]]; then
	echo ">>>>>>>>>>>>>>>>integration ROS2 PKG REBUILD FLAG ENABLED<<<<<<<<<<<<<<<"
	echo ">>>>>>>>>START REBUILDING AND INSTALLATION OF PKG 'integration'<<<<<<<<<"
	echo "    ____  __________  __  ________    ____  "
	echo "   / __ \/ ____/ __ )/ / / /  _/ /   / __ \ "
	echo "  / /_/ / __/ / __  / / / // // /   / / / / "
	echo " / _, _/ /___/ /_/ / /_/ // // /___/ /_/ /  "
	echo "/_/ |_/_____/_____/\____/___/_____/_____/   "
	colcon build \
		--build-base /root/ros_ws/build \
        --install-base /root/ros_ws/install \
        --base-paths /root/ros_ws/src \
		--symlink-install
elif [[ -n ${REBUILD_RPKG_GAZEBO} ]]; then
	echo ">>>>>>>>>>>>>>>>model_spawn ROS2 PKG REBUILD FLAG ENABLED<<<<<<<<<<<<<<<"
	echo ">>>>>>>>>START REBUILDING AND INSTALLATION OF PKG 'model_spawn'<<<<<<<<<"
	echo "    ____  __________  __  ________    ____  "
	echo "   / __ \/ ____/ __ )/ / / /  _/ /   / __ \ "
	echo "  / /_/ / __/ / __  / / / // // /   / / / / "
	echo " / _, _/ /___/ /_/ / /_/ // // /___/ /_/ /  "
	echo "/_/ |_/_____/_____/\____/___/_____/_____/   "
	colcon build \
		--build-base ~/gazebo_ros/build \
		--install-base ~/gazebo_ros/install \
		--base-paths ~/gazebo_ros/src \
		--symlink-install
fi

# Set environment variables in this shell script
source /root/gazebo_ros/install/setup.bash
source /root/px4_ros/install/setup.bash
source /root/ros_ws/install/setup.bash
source /usr/share/gazebo-11/setup.sh

# Run MAVLink Router for Communication with QGC
echo ">>>>>>>>>>>>>INITIALIZEING MAVLINK ROUTER FOR QGC CONNECTION<<<<<<<<<<<"
nohup mavlink-routerd -e 172.20.0.7:14550 127.0.0.1:14550 &
sleep 3s

# Run microRTPS bridge for Communication in ROS2 msg
echo ">>>>>>>>>>>INITIALIZEING microRTPS BRIDGE FOR ROS2 CONNECTION<<<<<<<<<<"
micrortps_agent -t UDP &
sleep 1s

if [[ -n ${DEBUG_ENTRYPOINT} ]]; then
	echo ">>>>>>>>>>ENTRYPOINT DEBUGGING ENABLED. DO NOT RUN ANY PROCESS<<<<<<<<<"
	echo "    ____  __________  __  ________"
	echo "   / __ \/ ____/ __ )/ / / / ____/"
	echo "  / / / / __/ / __  / / / / / __  "
	echo " / /_/ / /___/ /_/ / /_/ / /_/ /  "
	echo "/_____/_____/_____/\____/\____/   "
	echo ">>>>>>>>>>>>>>>>>>>>>>>>>>Noneun Ge Jeil JOAH<<<<<<<<<<<<<<<<<<<<<<<<<<"
	echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Debugging JOAH<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
else
	echo ">>>>>>>>>>>>>>>>RUNNING ITE SITL WITH DEFINED CONDITION<<<<<<<<<<<<<<<<"
	echo "   _________ _____   __________  ____  "
	echo "  / ____/   /__  /  / ____/ __ )/ __ \ "
	echo " / / __/ /| | / /  / __/ / __  / / / / "
	echo "/ /_/ / ___ |/ /__/ /___/ /_/ / /_/ /  "
	echo "\____/_/  |_/____/_____/_____/\____/   "
	HEADLESS=${HEADLESS} make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_${PX4_SIM_MODEL}__${PX4_SIM_WOLRLD} &
	sleep 1s
	# Spawn objects in gazebo world
	echo "Spawning Objects"
	ros2 run model_spawn ModelSpawn 100 20 &
	sleep 1s
	# Run integration node for the one and the all
	echo "Run integration node"
	ros2 run integration IntegrationTest &
fi

# Keep container running. The Sleeping Beauty
sleep infinity