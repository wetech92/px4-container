# A4-VAI ITE Container Image: Gazebo

- Please Refer to [Running a Container](/docs/runContainer.md) for more info

## 1 Manual Run

```shell
xhost +
```

### 1.1 Run Container

#### 1.1.1 Generic Linux System

```shell
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY
  -e QT_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e PX4_SIM_MODEL=typhoon_inha \
  -e PX4_SIM_WOLRLD=grass \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
  -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
  -v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  --net host \
  --device=/dev/dri:/dev/dri \
  --name gazebo-sitl \
  --gpus all \
  --privileged \
  kestr3l/px4:gazebo-gpu-0.0.3 zsh
```

#### 1.1.2 Windows Subsystems for Linux 2 (WSL2 / Windows 11)

```shell
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e QT_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e PX4_SIM_MODEL=typhoon_inha \
  -e PX4_SIM_WOLRLD=grass \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /usr/lib/wsl:/usr/lib/wsl \
  --device=/dev/dri:/dev/dri \
  --device=/dev/dxg \
  --name gazebo-sitl \
  --net host \
  --gpus all \
  --privileged \
  kestr3l/px4:gazebo-gpu-0.0.3 zsh
```

#### 1.1.3 QGroundControl

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=/tmp \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   --net host \
   --device=/dev/dri:/dev/dri \
   --privileged \
   kestr3l/qgc-app:nobg-4.0.0
```

### 1.2 Manual Commands

#### 1.2.1 Connect Terminal to Container

- Do this on each command execution

```shell
docker exec -it gazebo-sitl zsh
```

#### 1.2.2 Run microRTPS

```shell
micrortps_agent -t UDP
```

#### 1.2.3 Run microRTPS

```shell
make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_typhoon_inha__grass
```

> Model typhoon_inha / World grass

#### 1.2.4 Model Spawn Service

```shell
ros2 run model_spawn ModelSpawn 100 20
```

> Known object 100 / Unkown object 20

#### 1.2.4 Model Spawn Service

```shell
ros2 run integration IntegrationTest
```

## 2. Docker-Compose

```shell
docker-compose up
```

## 3. Expected Result

- Gazebo will start and QGC will be stated as 'Connected'
- Objects will be generated on map
- Soon after, flight will begin