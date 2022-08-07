ARG BASEIMAGE
ARG BASETAG

FROM ${BASEIMAGE}:${BASETAG} as stage_apt

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN \
    rm -rf /etc/apt/apt.conf.d/docker-clean \
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

ENV \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=galactic

COPY Base/aptdeps.txt /tmp/aptdeps.txt

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
    --mount=type=cache,target=/etc/apt/sources.list.d,from=stage_apt,source=/etc/apt/sources.list.d \
	apt-get install --no-install-recommends -y $(cat /tmp/aptdeps.txt) \
    && rm -rf /tmp/*

RUN \
	cd /usr/src/gtest \
	&& mkdir build && cd build \
	&& cmake .. && make -j$(nproc) \
	&& find . -name \*.a -exec cp {} /usr/lib \; \
	&& cd .. && rm -rf build

RUN \
    wget -q "https://services.gradle.org/distributions/gradle-6.8.3-bin.zip" -O /tmp/gradle-6.8.3-bin.zip \
	&& mkdir /opt/gradle \
	&& cd /tmp \
	&& unzip -d /opt/gradle gradle-6.8.3-bin.zip \
	&& rm -rf /tmp/*

ENV PATH "/opt/gradle/gradle-6.8.3/bin:$PATH"


FROM stage_deps as stage_foomemdds

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN \
    git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory \
	&& cd /tmp/foonathan_memory \
	&& mkdir build && cd build \
	&& cmake .. \
    && cmake --build . --target install -- -j $(nproc)

RUN \
    git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 /tmp/FastDDS-2.0.2 \
	&& cd /tmp/FastDDS-2.0.2 \
	&& mkdir build && cd build \
	&& cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
    && cmake --build . -- -j $(nproc)


FROM stage_deps as stage_fastrtps

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN \
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /tmp/Fast-RTPS-Gen-1.0.4 \
	&& cd /tmp/Fast-RTPS-Gen-1.0.4 \
	&& gradle assemble


FROM stage_deps as stage_python

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG PROCVER

ENV \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display

COPY Base/pydeps_cpu.txt  /tmp/pydeps_cpu.txt
COPY Base/pydeps_gpu.txt  /tmp/pydeps_gpu.txt
COPY scripts/pypkg.sh /tmp/pypkg.sh

RUN /tmp/pypkg.sh \
    && rm -rf /tmp/*


FROM stage_deps as stage_mid

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY --from=stage_foomemdds /tmp/foonathan_memory /tmp/foonathan_memory
RUN \
    cd /tmp/foonathan_memory/build \
    && cmake --install . \
    && rm -rf /tmp/*

COPY --from=stage_foomemdds /tmp/FastDDS-2.0.2 /tmp/FastDDS-2.0.2
RUN \
    cd /tmp/FastDDS-2.0.2/build \
    && cmake --install . \
    && rm -rf /tmp/*

COPY --from=stage_fastrtps /tmp/Fast-RTPS-Gen-1.0.4 /tmp/Fast-RTPS-Gen-1.0.4
RUN \
    cd /tmp/Fast-RTPS-Gen-1.0.4 \
    && gradle install \
    && rm -rf /tmp/*

COPY --from=stage_python /root/.local /root/.local
ENV PATH=/root/.local/bin:$PATH

RUN \
    echo source "/opt/ros/galactic/setup.bash" >> /root/.bashrc \
    && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc \
    && echo "export _colcon_cd_root=~/ros2_install" >> /root/.bashrc

# setup colcon mixin and metadata
RUN \
    colcon mixin add default \
		https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
	&& colcon mixin update \
	&& colcon metadata add default \
		https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
	&& colcon metadata update


FROM stage_mid as stage_px4

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN git clone -b v1.13.0 https://github.com/PX4/PX4-Autopilot.git /root/PX4-Autopilot \ 
	&& cd /root/PX4-Autopilot \
	&& git submodule update --init --recursive

COPY PX4-Autopilot /root/PX4-Autopilot
COPY px4_ros /tmp/px4_ros

RUN \
    git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros/src/px4_ros_com \
    && git clone https://github.com/PX4/px4_msgs.git ~/px4_ros/src/px4_msgs \
    && cp -r /tmp/px4_ros/src/* ~/px4_ros/src \
    && rm -rf /root/px4_ros/src/px4_msgs/msg/* \
        /root/px4_ros/src/px4_ros_com/src/examples \
        /root/px4_ros/src/px4_ros_com/launch \
    && python3 /root/PX4-Autopilot/msg/tools/uorb_to_ros_urtps_topics.py \
        -i /root/PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml \
        -o /root/px4_ros/src/px4_ros_com/templates/urtps_bridge_topics.yaml \
    && python3 /root/PX4-Autopilot/msg/tools/uorb_to_ros_msgs.py \
        /root/PX4-Autopilot/msg/ \
        /root/px4_ros/src/px4_msgs/msg/ \
    && rm -rf /tmp/*

RUN \
    source /opt/ros/galactic/setup.bash \
        && colcon build --build-base ~/px4_ros/build \
        --install-base ~/px4_ros/install \
        --base-paths ~/px4_ros/src \
        --symlink-install \
        --event-handlers console_direct+


FROM stage_mid as stage_mavr

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN git clone https://github.com/mavlink-router/mavlink-router.git /root/mavlink-router \
    && cd /root/mavlink-router \
    && git submodule update --init --recursive \
    && meson setup build . \
    && ninja -C build


FROM stage_mid as stage_final

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

COPY --from=stage_px4 /root/PX4-Autopilot /root/PX4-Autopilot
COPY --from=stage_px4 /root/px4_ros /root/px4_ros
COPY --from=stage_mavr /root/mavlink-router/build/src/mavlink-routerd /usr/bin/mavlink-routerd
COPY --from=stage_mavr /root/mavlink-router/build/mavlink-router.service /usr/lib/systemd/system/mavlink-router.service
 
RUN \
  useradd --shell /bin/bash -u 1001 -c "" -m user \
	&& usermod -a -G dialout user

RUN \
    mkdir /tmp/.X11-unix && \
	chmod 1777 /tmp/.X11-unix && \
	chown -R root:root /tmp/.X11-unix \
  && git config --global --add safe.directory "*"

ENV DISPLAY :99

ENV \
    CCACHE_UMASK=000 \
    FASTRTPSGEN_DIR="/usr/local/bin/" \
    PATH="/usr/lib/ccache:$PATH" \
    TERM=xterm \
    TZ=Asia/Seoul

EXPOSE 14556/udp
EXPOSE 14557/udp

CMD ["/bin/bash"]