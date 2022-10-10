# A4-VAI ILE PX4 Container Image: Gazebo

## 1. Build

### 1.1 CPU Only

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-cpu-<VERSION> \
    -t <IMAGE_NAME>:gazebo-cpu-<VERSION> \
    -f Gazebo/Dockerfile .
```

### 1.2 With GPU Support

- GPU-supported version includes `tensorflow-gpu 2.5.0` and `tf-agents 0.8.0`

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-gpu-<VERSION> \
    -t <IMAGE_NAME>:gazebo-gpu-<VERSION> \
    -f Gazebo/Dockerfile .
```

> Based on user's need, CUDA & cudNN's version may vary.<br/>
Check [nvidia/cuda](https://hub.docker.com/r/nvidia/cuda) for base imags with different CUDA & cudNN versions.

## 2. Run Container

- **Be aware that with `--rm` option, container will immediately be removed after process exit**
- However, it is very recommended to use it to avoid having leftover 'prune' containers

### 2.1 Basic Command

- Before entering command after boot, type `xhost +` first.
- For support of DRI (Direct Rendering Interface = H/W Acceleration), use following command:

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=/tmp \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   --net host \
   --device=/dev/dri:/dev/dri \
   --privileged \
   <IMAGE_NAME>:<TAG>
```

> If you are using Wayland as a display server protocol, (Ubuntu 22.034 or else) add following:<br/>
`-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY`

### 2.2 Rebuild User-Defined ROS2 Package and Run

- Giving environment variable `$REBUILD` as `true` will rebuild and install ROS2 packages in workspace `ros_ws`
- Bind mounting your modified workspace to `/root/ros_ws` will make test process easier without rebuilding container
- **THIS PART IS FOR PLACEHOLDER PURPORSE ONLY FOR NOW. IT IS NOT IMPLEMENTED YET. AS IMPLEMENTED, THIS LINE WILL BE REMOVED**

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=/tmp \
   -e REBUILD = true \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v <PATH_TO_YOUR_ROS_WORKSPACE>:/root/ros_ws \
   --net host \
   --device=/dev/dri:/dev/dri \
   --privileged \
   <IMAGE_NAME>:<TAG>
```

### 2.3 Start Container Without Running Simulation

- Our docker image does not have `ENTRYPOINT` option. Replacing `CMD` with other parameter will not initialize `entrypoint.sh`
- Declaring argument `CMD` as `bash` will simply initialize bash shell (`/bin/bash`)

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=/tmp \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   --net host \
   --device=/dev/dri:/dev/dri \
   --privileged \
   <IMAGE_NAME>:<TAG> bash
```

### 2.4 With WSLg GPU Acceleration Support

- WSL2 does support GPU acceleration via Direct-X **starting from Windows 11**
- Additional parameters should be handed over to use graphical H/W acceleration inside a docker container:
  - `-e LD_LIBRARY_PATH=/usr/lib/wsl/lib`
  - `-v /tmp/.X11-unix:/tmp/.X11-unix`
  - `-v /mnt/wslg:/mnt/wslg`
  - `-v /usr/lib/wsl:/usr/lib/wsl`
  - `--device=/dev/dxg \`
- **This is very recommended for WSL environment**

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
   -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v /mnt/wslg:/mnt/wslg \
   -v /usr/lib/wsl:/usr/lib/wsl \
   --device=/dev/dxg \
   --net host \
   --gpus all \
   --privileged \
   <IMAGE_NAME>:<TAG> bash
```

## 3. Usage
```shell
Must sequantially execute below each Terminal Number and Terminal Commands
Turn on each new 5 Terminals
Execute below Commands
```
```shell
docker exec -it Gzbash bash
```

### 3.1 Terminal-1 PX4-Autopilot

```shell
cd ~/PX4-Autopilot
make px4_sitl_rtps gazebo_{vehilce_option}__{world_option}
```

```shell
vehicle_option
Quadrotor: iris
Hexarotor: QTR_inha, typhoon_inha
```

```shell 
world_option
empty, grass, grass_dryden
```
### 3.2 Terminal-2 QGroundControl

```shell
cd 
su -c "/home/user/QGroundControl.AppImage --appimage-extract-and-run" user
```

```shell
Must Activate Check Application Settings/Virtual Joystick Checkbox
```
### 3.3 Terminal-3 FastRTPS-micrortps_agent

```shell
cd ~/px4_ros
. install/setup.bash
micrortps_agent -t UDP
```


### 3.4 Terminal-4 model_spawn package
```shell
cd ~/gazebo_ros
colcon build
. install/setup.bash
ros2 run model_spawn ModelSpawn {known_obs_num} {unknown_obs_num}
```

### 3.5 Terminal-5 integration package
```shell
cd ~/ros_ws
colcon build
. install/setup.bash
. ~/gazebo_ros/install/setup.bash
. ~/px4_ros/install/setup.bash
ros2 run integration IntegrationTest
```
