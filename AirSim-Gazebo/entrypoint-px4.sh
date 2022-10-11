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
	cd /root/ros_ws && colcon build
fi

# su -c "/home/user/ForestDeploy/ForestDeploy.sh -windowed" user &
# sleep 3s

nohup mavlink-routerd -e 172.21.0.7:14550 127.0.0.1:14550 &

source /opt/ros/galactic/setup.sh
source /root/px4_ros/install/setup.sh
source /root/ros_ws/install/setup.sh
source /root/AirSim/ros2/install/setup.sh
source /usr/share/gazebo-11/setup.sh

$build_path/bin/px4 -d "$build_path/etc" -w $build_path -s $build_path/etc/init.d-posix/rcS &
sleep 3s

gzserver ${sitl_gazebo_path}/worlds/${PX4_SIM_WOLRLD}.world &
sleep 3s

gz model \
	--spawn-file=${sitl_gazebo_path}/models/typhoon_inha/${PX4_SIM_MODEL}.sdf \
	--model-name=typhoon_inha -x 0.5 -y 0.5 -z -0.5 &
sleep 10s

/root/AirSim/GazeboDrone/build/GazeboDrone &
sleep 3s

ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=172.21.0.6 &

sleep infinity