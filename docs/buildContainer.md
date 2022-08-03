# Building a Container From Source

## Build Prequisites

- Host must have docker installed (>=19.03)
  - Recommend to install it by [convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script) provided by docker
- For GPU support, following conditions must be met.
  - Host must have Nvidia GPU with CUDA 11.3 support
  - Host must have Nvidia GPU Driver installed (=>418.71.07, >=Kepler)
    - On WSL, intall only on Windows host
  - Host must have [nvidia-docker-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed
    - Use nvidia-docker2
- Host must have at least 20 GB of smpty storage (Recommended)
- Host must be a X64 device

## Build Commands

### Build Command Basics

- This repository uses Docker Buildkit for faster, efficient build
   - Attempt to build image without `DOCKER_BUILDKIT` will result in error
- Basic form of command for building an image is as following:

```shell
$ DOCKER_BUILDKIT=1 docker build --no-cache \
--build-arg <ARG>=<VAR> \
-t <IMAGE_NAME>:<TAG> \
-f <TARGET_DOCKERFILE> .
```

- Suggested options have following definition:
   - `--no-cache`: Do not cache during build to save storage and suppress dangling images
   - `--build-arg <ARG>=<VAR>`: Designate ARG values for image build
      - Ex. `--build-arg BASEIMAGE=ubuntu --build-arg PROCVER=gpu`
      - As you can see, `--build-arg` can be stated multiple times
   - `-t <IMAGE_NAME>:<TAG>`: Designate image tag
      - Ex. `-t kestr3l/px4:base-cpu-0.0.2`
   - `-f`: Designate target dockerfile to build image
      - Ex. `-f /Base/Dockerfile`
   - `.`: Directory to start build. Which is, where to read files from

### Setting `ARG` values

- Three following `ARG` values must be set to start build process:
   - `BASEIMAGE`: Base image to build an image
      - For `base` tag, recommend to use `ubuntu:20.04` or `nvidia/cuda`
      - For case of `airsim` or `gazebo` tag, use prebuilt base image name
   - `BASETAG`: Base image's tag to build an image
      - Depends on type and image you want to build
   - `PROCVER`: Whether a image is GPU version of CPU version
      - Must be one from `gpu` or `cpu`
      - Value other than them will provoke a build error.

## Build Steps and Example

- Block diagrams of multi-stage build process are available in there as an image and draw.io file
- Build of `gazebo` and `airsim` tagged image requires corresponding `base` tagged image


### Base

- CPU only

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=ubuntu \
    --build-arg BASETAG=20.04 \
    --build-arg PROCVER=cpu \
    -t <IMAGE_NAME>:base-cpu-<VERSION> \
    -f Base/Dockerfile .
```

- With GPU support
  - GPU-supported version includes `tensorflow-gpu 2.5.0` and `tf-agents 0.8.0`

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=nvidia/cuda \
    --build-arg BASETAG=11.3.1-cudnn8-devel-ubuntu20.04 \
    --build-arg PROCVER=gpu \
    -t <IMAGE_NAME>:base-gpu-<VERSION> \
    -f Base/Dockerfile .
```

> Based on user's need, CUDA & cudNN's version may vary.
Check [nvidia/cuda](https://hub.docker.com/r/nvidia/cuda) for base imags with different CUDA & cudNN versions.

### Gazebo

- CPU only

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-cpu-<VERSION> \
    -t <IMAGE_NAME>:gazebo-cpu-<VERSION> \
    -f Gazebo/Dockerfile .
```

- With GPU support

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-gpu-<VERSION> \
    -t <IMAGE_NAME>:gazebo-gpu-<VERSION> \
    -f Gazebo/Dockerfile .
```

### AirSim

- CPU only

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-cpu-<VERSION> \
    -t <IMAGE_NAME>:airsim-cpu-<VERSION> \
    -f AirSim/Dockerfile .
```

> **Using a GPU is stronly recommended for AirSim simulatitin**
> UE4 simulation environment required a lot of graphical computing power to be run
> Building with command above will be run without any problem. However, it's not practical to use this image due to reason described above

- With GPU support

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=kestr3l/px4 \
    --build-arg BASETAG=base-gpu-0.0.2 \
    --build-arg VULKAN_SDK_VERSION=`curl -sk https://vulkan.lunarg.com/sdk/latest/linux.txt` \
    -t kestr3l/px4:airsim-gpu-0.0.2 \
    -f AirSim/Dockerfile .
```

## Reference

1. [Install Docker Engine on Ubuntu - Install using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2. [NVIDIA Driver Installation Quickstart Guide](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)
3. [NVIDIA CONTAINER TOOLKIT - Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
4. [Build images with Buildkit](https://docs.docker.com/develop/develop-images/build_enhancements/)