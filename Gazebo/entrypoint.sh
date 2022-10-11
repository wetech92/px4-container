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

source /opt/ros/galactic/setup.sh
source /root/gazebo_ros/install/setup.bash
source /root/px4_ros/install/setup.bash
source /root/ros_ws/install/setup.bash
source /usr/share/gazebo-11/setup.sh

# Run PX4 SITL
$build_path/bin/px4 -d "$build_path/etc" -w $build_path -s $build_path/etc/init.d-posix/rcS &

# Run Gazebo
if [ ${HEADLESS} -eq 1]; then
	echo "HEADLESS is ${HEADLESS}: 1, Running Gazebo SITL in HEADLESS mode"
	gzserver ${sitl_gazebo_path}/worlds/grass.world --verbose &
else
	echo "HEADLESS is ${HEADLESS}: Not 1, Running Gazebo SITL in normal mode"
	gazebo ${sitl_gazebo_path}/worlds/${PX4_SIM_WOLRLD}.world --verbose &
fi

sleep 2s

# Spawn Model
gz model \
	--spawn-file=${sitl_gazebo_path}/models/typhoon_inha/${PX4_SIM_MODEL}.sdf \
	--model-name=typhoon_inha -x 1.0 -y 1.0 -z 0.0 &
sleep 2s

nohup mavlink-routerd -e 172.20.0.7:14550 127.0.0.1:14550 &
sleep 3s

echo "Initializing microRTPS Bridge"
micrortps_agent -t UDP &
sleep 1s

echo "Spawning Objects"
ros2 run model_spawn ModelSpawn 100 20 &
sleep 1s

echo "Run integration node"
ros2 run integration IntegrationTest &
sleep infinity