FROM acsldocker/px4:base-0.0.1

COPY /scripts/docker-clean /etc/apt/apt.conf.d/docker-clean
COPY /scripts/sources.list /etc/apt/sources.list

# ---------- SET BASIC SETTINGS ----------
ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO foxy
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute,display

ENV SITL_MODEL iris
ENV SITL_ENV baylands
ENV GAZEBO_MASTER_URI http://localhost:11345
# http://$(hostname -I | cut -f1 -d' '):11345

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install --quiet -y \
		ros-$ROS_DISTRO-gazebo* \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*


COPY models /root/.gazebo/models

RUN apt-get -y --quiet --no-install-recommends install \
		ant \
		binutils \
		bc \
		dirmngr \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		libeigen3-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		mesa-utils \
		protobuf-compiler \
		x-window-system \
		ignition-edifice \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*


COPY PX4-Autopilot/Tools/sitl_gazebo/models /tmp/models

RUN HEADLESS=1 make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_${SITL_MODEL}__${SITL_ENV} \
    && cp -r /tmp/models ~/PX4-Autopilot/Tools/sitl_gazebo \
	&& rm -rf /tmp/*


COPY ROS2-node /root/ROS2-node

RUN colcon build \
		--build-base ~/ROS2-node/build \
		--install-base ~/ROS2-node/install \
		--base-paths ~/ROS2-node/src \
		--packages-select g_camera_lidar_sp \
	&& source ~/ROS2-node/install/setup.bash

WORKDIR /root

COPY QGroundControl.org /home/user/.config/QGroundControl.org
COPY Gazebo/entrypoint.sh /usr/local/bin/entrypoint.sh
CMD ["/usr/local/bin/entrypoint.sh"]