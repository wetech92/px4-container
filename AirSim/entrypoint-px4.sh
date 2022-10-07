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

if [ -n $PX4_SIM_HOST_ADDR ]; then
	find /root/AirSim/python -type f -name "*.py" -print0 | xargs -0 sed -i "s/airsim.VehicleClient()/airsim.VehicleClient(ip=\"${PX4_SIM_HOST_ADDR}\", port=41451)/g"
	find /root/AirSim/python -type f -name "*.py" -print0 | xargs -0 sed -i "s/airsim.MultirotorClient()/airsim.MultirotorClient(ip=\"${PX4_SIM_HOST_ADDR}\", port=41451)/g"
fi

# if ${WSL}; then
# 	find /root/AirSim/python -type f -name "*.py" -print0 | xargs -0 sed -i "s/airsim.VehicleClient()/airsim.VehicleClient(ip = str(os.environ\['simhost']), port=41451)/g"
# 	#find /root/AirSim/python -type f -name "*.py" -print0 | xargs -0 sed -i "s/ip = str(os.environ\['simhost']), port=41451//g" for reverse
# fi

rm -rf /root/shared/Map.png &
rm -rf /root/shared/simOn &
sleep 1s

simFlag=$(find /root/shared -maxdepth 1 -type f -name 'simOn')

while [ -z $simFlag ];
do
    simFlag=$(find /root/shared -maxdepth 1 -type f -name 'simOn')
	echo "Waiting until simulator starts up..."
	sleep 0.5s
done

echo "Simulator startup! Generating Objects"
python3 /root/AirSim/python/spawnObject.py -a SM_SP_01 -r 240 240
sleep 0.5s

python3 /root/AirSim/python/moveUAV.py

$build_path/bin/px4 -d "$build_path/etc" -w $build_path -s $build_path/etc/init.d-posix/rcS &
nohup mavlink-routerd -e 172.19.0.7:14550 127.0.0.1:14550 &
sleep 3s

source /opt/ros/galactic/setup.sh
source /root/AirSim/ros2/install/setup.bash
source /root/px4_ros/install/setup.bash
source /root/ros_ws/install/setup.bash

micrortps_agent -t UDP &
sleep 1s

echo "Starting AIRSIM ROS PKGS"
ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=172.19.0.5 &
sleep 1s

mapImg=$(find /root/shared -maxdepth 1 -type f -name '*.png')

while [ -z $mapImg ];
do
    mapImg=$(find /root/shared -maxdepth 1 -type f -name '*.png')
	echo "Finding generated map..."
	sleep 1s
done

echo "Found generated map! Copying to RRT directory"
sleep 1s

mkdir /root/ros_ws/src/integration/integration/PathPlanning/Map/
cp $mapImg /root/ros_ws/src/integration/integration/PathPlanning/Map/Map.png
sleep 5s

ros2 run integration IntegrationTest
sleep infinity