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
ENV PX4_SIM_HOST_ADDR 172.19.0.6
 
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN make -C /root/PX4-Autopilot px4_sitl_rtps none_${SITL_MODEL} &

COPY ROS2-node /root/ROS2-node

RUN colcon build \
		--build-base ~/ROS2-node/build \
		--install-base ~/ROS2-node/install \
		--base-paths ~/ROS2-node/src \
		--packages-select g_camera_lidar_sp \
	&& source ~/ROS2-node/install/setup.bash

RUN pip install msgpack-rpc-python \
	&& pip install airsim

WORKDIR /root

COPY QGroundControl.org /home/user/.config/QGroundControl.org
COPY AirSim/entrypoint.sh /usr/local/bin/entrypoint.sh
CMD [ "/usr/local/bin/entrypoint.sh" ]