# Building a Container From Source

## 1 Build Prequisites

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

## 2 Build Commands

### 2.1 Build Command Basics

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
      - Ex. `-t kestr3l/px4:base-gpu-0.0.2`
   - `-f`: Designate target dockerfile to build image
      - Ex. `-f /Base/Dockerfile`
   - `.`: Directory to start build. Which is, where to read files from

### 2.2 Setting `ARG` values

- Three following `ARG` values must be set to start build process:
   - `BASEIMAGE`: Base image to build an image
      - For `base` tag, recommend to use `ubuntu:20.04` or `nvidia/cuda`
      - For case of `airsim` or `gazebo` tag, use prebuilt base image name
   - `BASETAG`: Base image's tag to build an image
      - Depends on type and image you want to build
   - `PROCVER`: Whether a image is GPU version of CPU version
      - Must be one from `gpu`
      - Value other than them will provoke a build error.

## 3 Build Steps and Example

- Block diagrams of multi-stage build process are available in there as an image and draw.io file
- Build of `gazebo` and `airsim` tagged image requires corresponding `base` tagged image

### 3.1 Base

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

### 3.2 Gazebo

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=<IMAGE_NAME> \
    --build-arg BASETAG=base-gpu-<VERSION> \
    -t <IMAGE_NAME>:gazebo-gpu-<VERSION> \
    -f Gazebo/Dockerfile .
```

### 3.3 AirSim

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=kestr3l/px4 \
    --build-arg BASETAG=base-gpu-0.0.2 \
    --build-arg VULKAN_SDK_VERSION=`curl -sk https://vulkan.lunarg.com/sdk/latest/linux.txt` \
    -t <IMAGE_NAME>:airsim-gpu-<VERSION> \
    -f AirSim/Dockerfile .
```

### 3.4 AirSim-Gazebo

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg BASEIMAGE=kestr3l/px4 \
    --build-arg BASETAG=gazebo-gpu-0.0.2 \
    --build-arg VULKAN_SDK_VERSION=`curl -sk https://vulkan.lunarg.com/sdk/latest/linux.txt` \
    -t <IMAGE_NAME>:airsimg-gpu-<VERSION> \
    -f AirSim-Gazebo/Dockerfile .
```

> AirSim-Gazebo builds from the Gazebo image, not from the base image

## 4 Reference

1. [Install Docker Engine on Ubuntu - Install using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2. [NVIDIA Driver Installation Quickstart Guide](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)
3. [NVIDIA CONTAINER TOOLKIT - Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
4. [Build images with Buildkit](https://docs.docker.com/develop/develop-images/build_enhancements/)