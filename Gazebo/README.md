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
<br/>

### 1.2 With GPU Support

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-gpu-<VERSION> \
    -t <IMAGE_NAME>:gazebo-gpu-<VERSION> \
    -f Gazebo/Dockerfile .
```
<br/>

- GPU-supported version includes `tensorflow-gpu 2.5.0` and `tf-agents 0.8.0`

> Based on user's need, CUDA & cudNN's version may vary.<br/>
Check [nvidia/cuda](https://hub.docker.com/r/nvidia/cuda) for base imags with different CUDA & cudNN versions.
<br/>


### 2.2 Rebuild User-Defined ROS2 Package and Run

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=/tmp \
   -e rebuild = true \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v <PATH_TO_YOUR_ROS_WORKSPACE>:/root/ROS2-node \
   --net host \
   --privileged \
   <IMAGE_NAME>:<TAG>
```

### 2.3 Start Container Without Running Simulation

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=/tmp \
   -e rebuild = true \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v <PATH_TO_YOUR_ROS_WORKSPACE>:/root/ROS2-node \
   -v <PATH_OF_YOUR_MISC_WORKSPACE>:/root/tmp \
   --net host \
   --privileged \
   <IMAGE_NAME>:<TAG> bash
```

### 2.4 With WSLg GPU Acceleration Support

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