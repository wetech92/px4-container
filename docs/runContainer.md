# Running Integrated Training Environment Containers

## 1 Basics of docker container

### 1.1 Concept of container and its characteristics

![Docker Lifecycle](images/Docker_Lifecycle.png)

- Docker container is a **non-persistent, isolated virtual environment**
- Key point is that **files are volatile** unless saved in mapped volume or copied to local direcetory
- Docker containers are created from docker images. If you have an image, you can create unlimited numbers of containers from it.
  - Think of Linux `.iso` image. If you have it, you can install windows on any conputer you want.

### 1.2 Creating Container

- There are two major ways to create desired docker container form an image:
  - Docker Command-Line Interface (CLI, a.k.a. `docker run`)
  - Container Orchestration
<br/>

- CLI is mainly used to create a sole, simple container. Its a startpoint of learning how to use a docker
- Container Orchestration is used to control a complex network of containers in easy, simultaneous way
- Kubernetes (k8s) is a 'de-facto' standard of container orchestration
  - However, we don't need that kind of bleeding-edge tool for running a simulation environment
- Therefore, we will use *docker-compose* as a mean of running multiple containers in designated relations

## 2 Docker CLI

### 2.1 Introduction & Example

- Docker CLI is based on `docker run <target_image>` and many optional pararmeters:

```shell
$ docker run [OPTIONS] IMAGE[:TAG|@DIGEST] [COMMAND] [ARG...]
```

- Possible options include...
  - **Container run state**: Detached? of foreground?
  - **Container identification**: Name of a container
  - **Environment variables passed/set**
  - **Volume mapping**: Sharing volume of lib with host
  - **Run constraints**: Kernel access, device access and more
<br/>

- Docker run statements are complex but example argument will help better undertanding docker CLI statements:
- Following is an example of `docker run` command for running AirSim container and their explanations:
  - *This is just an long and boring 'textbook-like' example. Do not use this to run ITE container*

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
   -e NVIDIA_DISABLE_REQUIRE=1 \
   -e NVIDIA_DRIVER_CAPABILITIES=all \
   -v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
   -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
   -v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   --device=/dev/dri:/dev/dri \
   --net host \
   --gpus all \
   --privileged \
   --name AirBash \
   kestr3l/px4:airsim-gpu-0.0.2 zsh
```

|Statement|Definition|Misc.|
|:-|:-|:-|
|`-it`|interactive terminal:<br/>`-i`: Keep STDIN open even if not attached<br/>`-t`: Allocate a pseudo-tty|Autonyms of `-d`|
|`-e`|set environment variable||
|`-v`|volume mapping with host||
|`--device`|device attached to host|Not works in WSL. Need a [workaround](https://docs.microsoft.com/ko-kr/windows/wsl/connect-usb)|
|`--net`|network settings|Set as `host` not to use a virtual network|
|`--gpus`|select gpu devices|Nvidia only. Requires [Nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)|
|`--privileged`|give an extended kernel access permission||
|`--name`|set container name|Cannot be duplicate of an existing name|
|`kestr3l/px4:airsim-gpu-0.0.2`|Target image<br/>Name: kestr3l/px4<br/>Tag: airsim-gpu-0.0.2|all characters must be lowercase or dash|
|`zsh`|override command to be run after container generation (`CMD`)|Ignored if `ENTRYPOINT` is set|

- All of A4-VAI ITE containers are `ENTRYPOINT`-free. That is, all default initialization scerips can be overriden by `CMD` statement
- This is important for debugging purpose. If not, run container without overrriding `CMD`.

### 2.2 Gazebo

#### 2.2.1 Generic Linux System

```shell
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY
  -e QT_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e PX4_SIM_MODEL=typhoon_inha \
  -e PX4_SIM_WOLRLD=grass \
  -e NO_PXH=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
  -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
  -v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
  --net host \
  --device=/dev/dri:/dev/dri \
  --gpus all \
  --privileged \
  kestr3l/px4:gazebo-gpu-0.0.3 zsh
```

> - If you experience OCI runtime issue, you can consider removing `-e /dev/dri:/dev/dri`

#### 2.2.2 Windows Subsystems for Linux 2 (WSL2 / Windows 11)

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
  --net host \
  --gpus all \
  --privileged \
  kestr3l/px4:gazebo-gpu-0.0.3 zsh
```

> - GPU H/W acceleration support on WSL (WSLg) is available starting from Windows 11
> - If your WSL2 environment is based on Windows 10, upgrade it to Windows 11 or use native Linux instead
> - If you experience OCI runtime or driver issue, you can consider removing `-e /dev/dri:/dev/dri`

### 2.3 Airsim

#### 2.3.1 Generic Linux System

```shell
docker run -it --rm \
   -e DISPLAY=$DISPLAY \
   -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
   -e QT_NO_MITSHM=1 \
   -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
   -e NVIDIA_DISABLE_REQUIRE=1 \
   -e NVIDIA_DRIVER_CAPABILITIES=all \
   -v /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
   -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
   -v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   --device=/dev/dri:/dev/dri \
   --net host \
   --gpus all \
   --privileged \
   kestr3l/px4:airsim-gpu-0.0.2 zsh
```

> - User must run AirSim environment on native Linux with Nvidia GPU
> - This is becuse there is a vulkan compatibility issue on GPU driver and WSL2 environment.
> - If you experience OCI runtime or driver issue, you can consider removing `-e /dev/dri:/dev/dri`

## 3 Docker-Compose

### 3.1 Intalling docker-compose

> If you already installed docker-compsoe, you can skip this part

- Basically, `docker-compose` is a way of running container with predescribed 'script' in `.yml` format
- It is a very strong tool since we can run and configure a network of containers at once
- However, docker-compose is not a default element of a docker engine. Therefore, it must be installed separately
<br/>

- First, install docker-compose binary from the repository. Currently, 2.9.0 is the latest version
- Make it executable and create a symbolic link for binary execution

```shell
sudo curl -L "https://github.com/docker/compose/releases/download/v2.11.2/docker-compose-linux-x86_64" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose
```

- After installation, check the version. Due to GPU access feature, version must be same or higher than `v1.28.0` 

```shell
docker-compose -v
# Docker Compose version v2.11.2 expected
```

### 3.2 Inspecting `docker-compose.yml`

- `docker-compose.yml` includes all settings required to create a set of containers
- For example, `docker-compose.yml` for AirSim instance is written as below:
  - Again, this is just an example. Just understanding it's structure is enough
  - For SITL purpose, please refer to the github repo

```yaml
version: "3"
services:
  airsim:
    privileged: true
    environment:
      - DISPLAY=$DISPLAY
      - WAYLAND_DISPLAY=$WAYLAND_DISPLAY
      - QT_NO_MITSHM=1 
      - XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
      - NVIDIA_DISABLE_REQUIRE=1
      - NVIDIA_DRIVER_CAPABILITIES=all 
      #- PX4_SIM_HOST_ADDR=192.168.219.86
      #- LD_LIBRARY_PATH=/usr/lib/wsl/lib
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /usr/share/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json
      - /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json 
      - /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json
      - /home/merlin/gitcodes/px4-container/AirSim/models/typhoon_h480/TyphoonH480.json:/home/user/Documents/AirSim/settings.json
    networks:
      px4-sitl-airsim:
        ipv4_address: 172.19.0.5
    container_name: sitl-airsim
    image: kestr3l/px4:airsim-gpu-0.0.2
    command: su -c "/home/user/ForestDeploy/ForestDeploy.sh -windowed" user
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
  px4:
    privileged: true
    environment:
      #- DISPLAY=$DISPLAY
      #- WAYLAND_DISPLAY=$WAYLAND_DISPLAY
      #- QT_NO_MITSHM=1 
      #- XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
      - PX4_SIM_HOST_ADDR=172.19.0.5
      #- LD_LIBRARY_PATH=/usr/lib/wsl/lib
    #volumes:
      #- '/tmp/.X11-unix:/tmp/.X11-unix'
      #- /mnt/wslg:/mnt/wslg
      #- /usr/lib/wsl:/usr/lib/wsl
    networks:
      px4-sitl-airsim:
        ipv4_address: 172.19.0.6
    container_name: sitl-px4
    image: kestr3l/px4:airsim-gpu-0.0.2
    command: zsh
    stdin_open: true
    tty: true
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
  qgc:
    privileged: true
    environment:
      - DISPLAY=$DISPLAY
      - WAYLAND_DISPLAY=$WAYLAND_DISPLAY
      - QT_NO_MITSHM=1 
      - XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
      #- PX4_SIM_HOST_ADDR=192.168.219.86
      #- LD_LIBRARY_PATH=/usr/lib/wsl/lib
    volumes:
      - '/tmp/.X11-unix:/tmp/.X11-unix'
      #- /mnt/wslg:/mnt/wslg
      #- /usr/lib/wsl:/usr/lib/wsl
    devices:
      - /dev/dri:/dev/dri
      #- /dev/dxg:/dev/dxg
    networks:
      px4-sitl-airsim:
        ipv4_address: 172.19.0.7
    container_name: sitl-qgc
    image: kestr3l/qgc-app:4.0.0
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
networks:
    px4-sitl-airsim:

```

- Except for the GPU allocation part, all settings in `docker-compose.yml` can be matched into statements of docker CLI
  - For example, `environment:` goes to `-e`, and `volume:` goes to `-v`
- However, it is much more intuitive in structure since each types of settings are grouped together
- Moreover, `docker-compose` can run and manage multiple containers at once
  - Example above runs 3 containers at once: AirSim, PX4-Autopilot and QGroundControl.
- As you can see, docker-compose is a very powerful tool for debugging purpose
  - You just simply need to mount your workspace into the container and run it. The virtual environment.

### 3.3 Starting and Killing docker-compose instance

- Basic docker-compose command is as following:

```shell
docker compose [-f <arg>...] [--profile <name>...] [options] [COMMAND] [ARGS...]
```

- If you want to run a predefined docker-compose instance, just use a following command in a directory including `docker-compose.yml`
  - `-d` means that containers will be run in background mode. So that it won't interrupted by a terminal exit

```shell
docker-compsoe up -d
```

- For docker-compose files other than `docker-compose.yml`, modify command as below:
  - Both relative and absolute directory works

```shell
docker-compsoe up -f <docker-compose-file> -d
```

- Use `docker-compose ps` to check running instance based on given `docker-compose.yml`
- `docker-compose down` will kill and clear up running docker-compose instance
  - **IT IS VERY RECOMMENDED TO DO SO AFTER THE RUN**
  - If skipped, there might be leftover temporary files in docker volume or etc.

## 4 Reference

1. [Docker 기초 (4) - 컨테이너 라이프사이클, 명령어](https://velog.io/@ghdud0503/Docker-%EA%B8%B0%EC%B4%88-3-%EC%BB%A8%ED%85%8C%EC%9D%B4%EB%84%88-%EB%9D%BC%EC%9D%B4%ED%94%84%EC%82%AC%EC%9D%B4%ED%81%B4)
2. [Docker docs: Docker run reference](https://docs.docker.com/engine/reference/run/)
3. [WSL 설명서: USB 디바이스 연결](https://docs.microsoft.com/ko-kr/windows/wsl/connect-usb)
4. [microsoft/wslg: Containerizing GUI applications with WSLg](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)
5. [Nvidia Container Toolkit: Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
6. [[Docker-Compose] 도커 컴포즈 설치 및 사용](https://soyoung-new-challenge.tistory.com/73)
7. [Docker docs: Overview of docker compose CLI](https://docs.docker.com/compose/reference/)