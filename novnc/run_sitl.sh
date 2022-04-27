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

# Run QGC & Gazebo SITL
if [ ${HEADLESS} -eq 1]; then
	echo "HEADLESS is ${HEADLESS}: 1, Running Gazebo SITL in HEADLESS mode"
	nohup su -c /home/user/QGroundControl.AppImage user & \
		HEADLESS=${HEADLESS} make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_${SITL_MODEL}__${SITL_ENV}
else
	echo "HEADLESS is ${HEADLESS}: Not 1, Running Gazebo SITL in normal mode"
	nohup su -c /home/user/QGroundControl.AppImage user & \
		make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_${SITL_MODEL}__${SITL_ENV}
fi