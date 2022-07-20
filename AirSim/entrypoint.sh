#!/bin/bash

# Start virtual X server in the background
# - DISPLAY default is :99, set in dockerfile
# - Users can override with `-e DISPLAY=` in `docker run` command to avoid
#   running Xvfb and attach their screen
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
	echo "Starting Xvfb"
	Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# # Check if the ROS_DISTRO is passed and use it
# # to source the ROS environment
# if [ -n "${ROS_DISTRO}" ]; then
# 	source "/opt/ros/$ROS_DISTRO/setup.bash"
# fi

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

if ${REBUILD}; then
	colcon build \
		--build-base /root/ROS2-node/build \
        --install-base /root/ROS2-node/install \
        --base-paths /root/ROS2-node/src \
	&& source /root/ROS2-node/install/setup.bash
fi

# Run ROS2 Nodes
source /root/AirSim/ros2/install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=$simhost

# Run QGC & Gazebo SITL
nohup su -c "/home/user/QGroundControl.AppImage --appimage-extract-and-run" user & \
	make -C /root/PX4-Autopilot px4_sitl_rtps none_${SITL_MODEL}