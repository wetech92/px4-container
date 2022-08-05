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
	/root/tools/buildRos2Pkg.sh
fi

if ${WSL}; then
	find /root/AirSim/python -type f -name "*.py" -print0 | xargs -0 sed -i "s/airsim.VehicleClient()/airsim.VehicleClient(ip = str(os.environ\['simhost']), port=41451)/g"
	#find /root/AirSim/python -type f -name "*.py" -print0 | xargs -0 sed -i "s/ip = str(os.environ\['simhost']), port=41451//g" for reverse
fi

su -c "/home/user/ForestDeploy/ForestDeploy.sh -windowed" user &
make -C /root/PX4-Autopilot px4_sitl_rtps none_typhoon_inha &
sleep 3s

micrortps_agent -t UDP &
ros2 launch airsim_ros_pkgs airsim_node.launch.py
# ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=$simhost &
# ros2 run integration IntegrationTest