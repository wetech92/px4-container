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


if ${REBUILD}; then
	colcon build \
		--build-base /root/ROS2-node/build \
        --install-base /root/ROS2-node/install \
        --base-paths /root/ROS2-node/src \
	&& source /root/ROS2-node/install/setup.bash
fi


# Run Gazebo SITL
if [ ${HEADLESS} -eq 1]; then
	echo "HEADLESS is ${HEADLESS}: 1, Running Gazebo SITL in HEADLESS mode"
	HEADLESS=${HEADLESS} make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_typhoon_inha__grass &
else
	echo "HEADLESS is ${HEADLESS}: Not 1, Running Gazebo SITL in normal mode"
	HEADLESS=${HEADLESS} make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_typhoon_inha__grass &
fi

sleep 120s

micrortps_agent -t UDP &
ros2 run model_spawn ModelSpawn