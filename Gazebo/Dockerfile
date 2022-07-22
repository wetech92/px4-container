ARG BASEIMAGE=kestr3l/px4
ARG BASETAG

FROM ${BASEIMAGE}:${BASETAG} as stage_apt

ENV \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
	&& echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache \
	&& apt-get update

RUN \
    apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt update \
    && apt install -y curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
        $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update


FROM ${BASEIMAGE}:${BASETAG} as stage_deps

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

COPY Gazebo/aptdeps.txt /tmp/aptdeps.txt

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
    --mount=type=cache,target=/etc/apt/sources.list.d,from=stage_apt,source=/etc/apt/sources.list.d \
	apt-get install --no-install-recommends -y $(cat /tmp/aptdeps.txt) \
    && rm -rf /tmp/*


FROM stage_deps as stage_rnode

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY Gazebo/tmp/gazebo_ros /root/gazebo_ros
COPY Gazebo/tmp/ros_ws /root/ros_ws


RUN \
	source "/opt/ros/galactic/setup.bash" \
	colcon build \
		--build-base ~/ros_ws/build \
		--install-base ~/ros_ws/install \
		--base-paths ~/ros_ws/src \
	&& colcon build \
		--build-base ~/gazebo_ros/build \
		--install-base ~/gazebo_ros/install \
		--base-paths ~/gazebo_ros/src
		

FROM stage_deps as stage_px4sitl

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

COPY Gazebo/tmp/PX4-Autopilot /root/PX4-Autopilot

RUN \
	make -C /root/PX4-Autopilot px4_sitl_rtps


FROM stage_deps as stage_final

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

ENV \
	DEBIAN_FRONTEND=noninteractive \
	LANG=C.UTF-8 \
	LC_ALL=C.UTF-8 \
	ROS_DISTRO=galactic \
	QT_X11_NO_MITSHM=1 \
	NVIDIA_VISIBLE_DEVICES=all \
	NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display
	# PX4_SIM_HOST_ADDR=localhost \
	# SITL_MODEL=iris \
	# SITL_ENV=baylands \
	# GAZEBO_MASTER_URI=http://localhost:11345 \
	# rebuild=false \
	# PATH=/root/.local/bin:$PATH
	# http://$(hostname -I | cut -f1 -d' '):11345

COPY --from=stage_rnode /root/gazebo_ros /root/gazebo_ros
COPY --from=stage_rnode /root/ros_ws /root/ros_ws
COPY --from=stage_px4sitl /root/PX4-Autopilot /root/PX4-Autopilot

COPY Gazebo/tmp/PX4-Autopilot/Tools/sitl_gazebo/models /root/.gazebo/models

RUN \
	chown -R user:user /home/user

WORKDIR /root

COPY QGroundControl.org /home/user/.config/QGroundControl.org
COPY scripts/initRepo.sh /root/tools/initRepo.sh

COPY Gazebo/entrypoint.sh /usr/local/bin/entrypoint.sh
CMD ["/usr/local/bin/entrypoint.sh"]