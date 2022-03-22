FROM nvidia/cuda:11.2.2-cudnn8-devel-ubuntu20.04

COPY /scripts/docker-clean /etc/apt/apt.conf.d/docker-clean
COPY /scripts/sources.list /etc/apt/sources.list

# ---------- INSTALL Basic dependencies ----------
ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO foxy
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute,display

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
		bzip2 \
		ca-certificates \
		ccache \
		cmake \
		cppcheck \
		curl \
		dirmngr \
		doxygen \
		file \
		g++ \
		gcc \
		gdb \
		git \
		gnupg \
		gosu \
		gstreamer1.0-gl \
		lcov \
		libfreetype6-dev \
		libgtest-dev \
		libpng-dev \
		libssl-dev \
		lsb-release \
		make \
		ninja-build \
		openjdk-11-jdk \
		openjdk-11-jre \
		openssh-client \
		pkg-config \
		python3-dev \
		python3-pip \
		rsync \
		shellcheck \
		tzdata \
		unzip \
		valgrind \
		wget \
		xsltproc \
		zip \
        fuse \
		libglvnd0 \
		libgl1 \
		libglx0 \
		libegl1 \
		libxext6 \
		libx11-6 \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# gtest
RUN cd /usr/src/gtest \
	&& mkdir build && cd build \
	&& cmake .. && make -j$(nproc) \
	&& find . -name \*.a -exec cp {} /usr/lib \; \
	&& cd .. && rm -rf build

# Install Python 3 pip build dependencies first.
RUN python3 -m pip install --upgrade pip wheel setuptools

# Python 3 dependencies installed by pip
RUN python3 -m pip install argparse argcomplete coverage cerberus empy jinja2 kconfiglib \
		matplotlib==3.0.* numpy nunavut>=1.1.0 packaging pkgconfig pyros-genmsg pyulog \
		pyyaml requests serial six toml psutil pyulog wheel jsonschema pynacl \
		tensorflow-gpu==2.5.0 tf-agents==0.8.0

# astyle v3.1
RUN wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
	&& cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
	&& make -f ../build/gcc/Makefile -j$(nproc) && cp bin/astyle /usr/local/bin \
	&& rm -rf /tmp/*

# Gradle (Required to build Fast-RTPS-Gen)
RUN wget -q "https://services.gradle.org/distributions/gradle-6.8.3-bin.zip" -O /tmp/gradle-6.8.3-bin.zip \
	&& mkdir /opt/gradle \
	&& cd /tmp \
	&& unzip -d /opt/gradle gradle-6.8.3-bin.zip \
	&& rm -rf /tmp/*

ENV PATH "/opt/gradle/gradle-6.8.3/bin:$PATH"

# Intall foonathan_memory from source as it is required to Fast-RTPS >= 1.9
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory \
	&& cd /tmp/foonathan_memory \
	&& mkdir build && cd build \
	&& cmake .. \
	&& cmake --build . --target install -- -j $(nproc) \
	&& rm -rf /tmp/*

# Fast-DDS (Fast-RTPS 2.0.2)
RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 /tmp/FastDDS-2.0.2 \
	&& cd /tmp/FastDDS-2.0.2 \
	&& mkdir build && cd build \
	&& cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
	&& cmake --build . --target install -- -j $(nproc) \
	&& rm -rf /tmp/*

# Fast-RTPS-Gen 1.0.4
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /tmp/Fast-RTPS-Gen-1.0.4 \
	&& cd /tmp/Fast-RTPS-Gen-1.0.4 \
	&& gradle assemble \
	&& gradle install \
	&& rm -rf /tmp/*

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user \
	&& usermod -a -G dialout user

# setup virtual X server
RUN mkdir /tmp/.X11-unix && \
	chmod 1777 /tmp/.X11-unix && \
	chown -R root:root /tmp/.X11-unix
ENV DISPLAY :99

ENV CCACHE_UMASK=000
ENV FASTRTPSGEN_DIR="/usr/local/bin/"
ENV PATH="/usr/lib/ccache:$PATH"
ENV TERM=xterm
ENV TZ=Asia/Seoul

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# ---------- INSTALL ROS2-foxy ----------

# setup ros2 keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# setup sources.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install bootstrap tools
RUN apt-get update && apt-get install --quiet --no-install-recommends -y \
		python3-colcon-common-extensions \
		python3-colcon-mixin \
		python3-rosdep \
		python3-vcstool \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# install ros2 desktop
RUN apt-get update && apt-get install --quiet -y \
		ros-$ROS_DISTRO-desktop \
		ros-$ROS_DISTRO-launch-testing-ament-cmake \
		ros-$ROS_DISTRO-ros2bag \
		ros-$ROS_DISTRO-rosidl-generator-dds-idl \
	&& echo source "/opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
	&& echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
    && echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists /tmp/* /var/tmp/*

# Install Python 3 packages needed for testing
RUN pip3 install -U \
		argcomplete \
		flake8 \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures

# setup colcon mixin and metadata
RUN colcon mixin add default \
		https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
	&& colcon mixin update \
	&& colcon metadata add default \
		https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
	&& colcon metadata update

# ---------- PX4 and designed ROS2 nodes ----------

# Clone PX4-Autopilot and build once. Then, overwrite model directory
COPY PX4-Autopilot /tmp/PX4-Autopilot

RUN git clone -b v1.12.3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot \
	&& cp -r /tmp/PX4-Autopilot/msg ~/PX4-Autopilot \    
	&& cd ~/PX4-Autopilot \
	&& git submodule update --init --recursive \
	&& rm -rf /tmp/*

# INSTALL px4_ros_com
COPY px4_ros_com_ros2 /tmp/px4_ros_com_ros2

RUN git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com \
	&& git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs \
	&& cp -r /tmp/px4_ros_com_ros2/src/px4_msgs/msg ~/px4_ros_com_ros2/src/px4_msgs \
    && cp -r /tmp/px4_ros_com_ros2/src/rtps_command ~/px4_ros_com_ros2/src \
	&& source /opt/ros/foxy/setup.bash \
    && colcon build \
		--build-base ~/px4_ros_com_ros2/build \
		--install-base ~/px4_ros_com_ros2/install \
		--base-paths ~/px4_ros_com_ros2/src \
		--symlink-install \
		--event-handlers \
		console_direct+ \
    && source /opt/ros/foxy/setup.bash \
	&& rm -rf /tmp/*

# Download QGC and make it executable
RUN wget -q "https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage" \
		-O /home/user/QGroundControl.AppImage \
    && chmod +x /home/user/QGroundControl.AppImage


WORKDIR /root

COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
CMD ["/usr/local/bin/entrypoint.sh"]