ARG BASEIMAGE=kestr3l/px4
ARG BASETAG

FROM ${BASEIMAGE}:${BASETAG} as stage_apt

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

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

COPY AirSim/aptdeps.txt /tmp/aptdeps.txt

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
    --mount=type=cache,target=/etc/apt/sources.list.d,from=stage_apt,source=/etc/apt/sources.list.d \
	apt-get install --no-install-recommends -y $(cat /tmp/aptdeps.txt)


FROM stage_deps as stage_airsim

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

COPY AirSim/pydeps.txt /tmp/pydeps.txt

RUN \
	python3 -m pip install --user \
		$(sed '$d' /tmp/pydeps.txt) \
	&& python3 -m pip install --user \
		$(sed '$!d' /tmp/pydeps.txt)

RUN git clone https://github.com/microsoft/AirSim.git -b v1.8.1 /root/AirSim \
	&& /root/AirSim/setup.sh \
	&& /root/AirSim/build.sh

COPY AirSim/ros2/airsim_ros_wrapper.cpp /root/AirSim/ros2/src/airsim_ros_pkgs/src/airsim_ros_wrapper.cpp

WORKDIR /root/AirSim/ros2

RUN source /opt/ros/galactic/setup.bash \
	&& colcon build \
		--cmake-args -DCMAKE_C_COMPILER=gcc-8 \
		--cmake-args -DCMAKE_CXX_COMPILER=g++-8 \
		--cmake-args -DCMAKE_BUILD_TYPE=Debug


FROM stage_deps as stage_rnode

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

COPY AirSim/ros_ws /root/ros_ws

RUN colcon build \
		--build-base ~/ros_ws/build \
		--install-base ~/ros_ws/install \
		--base-paths ~/ros_ws/src \
		--symlink-install


FROM stage_deps as stage_px4sitl

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

RUN \
	make -C /root/PX4-Autopilot px4_sitl_rtps


FROM stage_deps as stage_final

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV \
	DEBIAN_FRONTEND noninteractive \
	LANG=C.UTF-8 \
	LC_ALL=C.UTF-8  \
	ROS_DISTRO=galactic \
	QT_X11_NO_MITSHM=1 \
	NVIDIA_VISIBLE_DEVICES=all \
	NVIDIA_DRIVER_CAPABILITIES=all \
	PATH=/root/.local/bin:$PATH \
	SITL_MODEL=typhoon_inha

COPY --from=stage_airsim /root/AirSim/AirLib /root/AirSim/AirLib
COPY --from=stage_airsim /root/AirSim/MavLinkCom /root/AirSim/MavLinkCom
COPY --from=stage_airsim /root/AirSim/cmake /root/AirSim/cmake
COPY --from=stage_airsim /root/AirSim/external /root/AirSim/external
COPY --from=stage_airsim /root/AirSim/ros2 /root/AirSim/ros2
COPY --from=stage_airsim /root/.local /root/.local

COPY --from=stage_rnode /root/ros_ws /root/ros_ws

COPY --from=stage_px4sitl /root/PX4-Autopilot /root/PX4-Autopilot

COPY AirSim/python /root/AirSim/python
COPY scripts /root/tools
COPY AirSim/scripts /root/run

COPY --chown=1001:1001 AirSim/ForestDeploy /home/user/ForestDeploy
COPY --chown=1001:1001 AirSim/models/typhoon_h480/TyphoonH480.json /home/user/Documents/AirSim/settings.json

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
    --mount=type=cache,target=/etc/apt/sources.list.d,from=stage_apt,source=/etc/apt/sources.list.d \
	sh -c "$(wget -O - https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh | sed "77d;s/export TERM=xterm/export TERM=xterm-256color/g")" -- \
        -t agnoster \
        -p git \
        -p https://github.com/zsh-users/zsh-autosuggestions \
        -p https://github.com/zsh-users/zsh-syntax-highlighting \
	&& echo "alias cat='batcat --paging=never'" >> /root/.zshrc \
	&& sed -i "s/#Port 22/Port 1022/g" /etc/ssh/sshd_config \
	&& sed -i "s/#PermitRootLogin prohibit-password/PermitRootLogin yes/g" /etc/ssh/sshd_config \
	&& sed -i "s/#PasswordAuthentication yes/PasswordAuthentication yes/g" /etc/ssh/sshd_config \
	&& chmod +x /home/user/ForestDeploy/ForestDeploy.sh

RUN \
	echo "source /root/AirSim/ros2/install/setup.bash" >> /root/.bashrc \
	&& echo "source /root/px4_ros/install/setup.bash" >> /root/.bashrc \
	&& echo "source /root/ros_ws/install/setup.bash" >> /root/.bashrc

RUN \
	echo "source /root/AirSim/ros2/install/setup.zsh" >> /root/.zshrc \
	&& echo "source /root/px4_ros/install/setup.zsh" >> /root/.zshrc \
	&& echo "source /root/ros_ws/install/setup.zsh" >> /root/.zshrc

WORKDIR /root

COPY AirSim/entrypoint-sim.sh /usr/local/bin/entrypoint.sh
# COPY AirSim/entrypoint-px4.sh /usr/local/bin/entrypoint.sh
CMD [ "/usr/local/bin/entrypoint.sh" ]
