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

COPY Gazebo/aptdeps.txt /tmp/aptdeps.txt

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
    --mount=type=cache,target=/etc/apt/sources.list.d,from=stage_apt,source=/etc/apt/sources.list.d \
	apt-get install --no-install-recommends -y $(cat /tmp/aptdeps.txt) \
    && rm -rf /tmp/*


FROM stage_deps as stage_rnode

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY Gazebo/gazebo_ros /root/gazebo_ros
COPY Gazebo/ros_ws /root/ros_ws

RUN \
	source /opt/ros/galactic/setup.bash \
	&& colcon build \
		--build-base ~/ros_ws/build \
		--install-base ~/ros_ws/install \
		--base-paths ~/ros_ws/src \
		--symlink-install \
	&& colcon build \
		--build-base ~/gazebo_ros/build \
		--install-base ~/gazebo_ros/install \
		--base-paths ~/gazebo_ros/src \
		--symlink-install
		

FROM stage_deps as stage_px4sitl

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

COPY Gazebo/models /root/PX4-Autopilot/Tools/sitl_gazebo/models
COPY Gazebo/worlds /root/PX4-Autopilot/Tools/sitl_gazebo/worlds
COPY Gazebo/plugins /root/PX4-Autopilot/Tools/sitl_gazebo

RUN \
	HEADLESS=1 make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_typhoon_inha__grass


FROM stage_deps as stage_final

SHELL ["/bin/bash", "-o", "pipefail", "-c"] 

ENV \
	DEBIAN_FRONTEND=noninteractive \
	LANG=C.UTF-8 \
	LC_ALL=C.UTF-8 \
	ROS_DISTRO=galactic \
	QT_X11_NO_MITSHM=1 \
	NVIDIA_VISIBLE_DEVICES=all \
	NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
	REBUILD=false \
	PATH=/root/.local/bin:$PATH

COPY --from=stage_rnode /root/gazebo_ros /root/gazebo_ros
COPY --from=stage_rnode /root/ros_ws /root/ros_ws
COPY --from=stage_px4sitl /root/PX4-Autopilot /root/PX4-Autopilot

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
    --mount=type=cache,target=/etc/apt/sources.list.d,from=stage_apt,source=/etc/apt/sources.list.d \
        sh -c "$(wget -O - https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh | sed "77d;s/export TERM=xterm/export TERM=xterm-256color/g")" -- \
        -t agnoster \
        -p git \
		-p https://github.com/zsh-users/zsh-autosuggestions \                                                                                                                                     -p https://github.com/zsh-users/zsh-autosuggestions \
        -p https://github.com/zsh-users/zsh-syntax-highlighting \
        && echo "alias cat='batcat --paging=never'" >> /root/.zshrc \
	&& sed -i "s/#Port 22/Port 1022/g" \
		/etc/ssh/sshd_config \
	&& sed -i "s/#PermitRootLogin prohibit-password/PermitRootLogin yes/g" \
		/etc/ssh/sshd_config \
	&& sed -i "s/#PasswordAuthentication yes/PasswordAuthentication yes/g" \
		/etc/ssh/sshd_config \
	&& sed -i "s/\${GAZEBO_PLUGIN_PATH}/\/root\/PX4-Autopilot\/build\/px4_sitl_rtps\/build_gazebo/g" \
		/usr/share/gazebo-11/setup.sh \
	&& sed -i "s/\${GAZEBO_MODEL_PATH}/\/root\/PX4-Autopilot\/Tools\/sitl_gazebo\/models/g" \
		/usr/share/gazebo-11/setup.sh \
	&& ln -s /root/PX4-Autopilot/Tools/sitl_gazebo/models /root/models

RUN \
	echo "source /usr/share/gazebo-11/setup.sh" >> /root/.bashrc \
	&& echo "source /root/px4_ros/install/setup.bash" >> /root/.bashrc \
	&& echo "source /root/ros_ws/install/setup.bash" >> /root/.basrhc \
    && echo "source /root/gazebo_ros/install/setup.bash" >> /root/.bashrc

RUN \
	echo "source /usr/share/gazebo-11/setup.sh" >> /root/.zshrc \
	&& echo "source /root/px4_ros/install/setup.zsh" >> /root/.zshrc \
	&& echo "source /root/ros_ws/install/setup.zsh" >> /root/.zshrc \
	&& echo "source /root/gazebo_ros/install/setup.zsh" >> /root/.zshrc

WORKDIR /root

COPY scripts/initRepo.sh /root/tools/initRepo.sh

COPY Gazebo/entrypoint.sh /usr/local/bin/entrypoint.sh
CMD ["/usr/local/bin/entrypoint.sh"]