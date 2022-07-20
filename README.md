# A4-VAI 통합 시뮬레이터 환경 PX4 컨테이너 이미지

- **주의! 이미지 빌드 방법이 많이 변경되어 README를 업데이트 중입니다!**
- ***이미지 업데이트가 완료되면 본 문장은 사라집니다.**

## 1. Introduction

### 1.1 What is this repository for?

- This is a repository containing files to build `kestr3l/px4` images
- Those images were built to implement Integrated Learning Environment Simulator
<br/>

### 1.2 Image Tags

- There are three matjor types of images:
   - `kestr3l/px4:base`: Base image including dependcies to run PX4-Autopilot and ROS2-Galactic
      -Base image of `airsim` and `gazebo` image
   - `kestr3l/px4:airsim`: Image to run PX4 AirSim SITL
   - `kestr3l/px4:gazebo`: Image to run PX4 Gazebo SITL
<br/>

- Minor tags of image are as following:
   - `gpu`: Image for Machine Learning, Reinforced Learning and etc.
      - Based on `nvidia/cuda:11.2.2-cudnn8-devel-ubuntu20.04`
   - `cpu`: Image withut GPU support. Intended to be used on device with no GPU
      - Based on `ubuntu:20.04`
<br/>

- Naming rule of image is as `kestr3l/px4:\<MAJOR\>-\<MINOR\>-\<VERSION\>`
   - Example: `kestr3l/px4:airsim-gpu-0.0.2`
   - Image for AirSim SITL with GPU support, version 0.0.2
<br/>

### 1.3 Branches and Miscellaneous

- Currently, we have two branches in this repository
   - `main`: Main, stable branch. Check tag and `CHANGELOG.md `for verision info 
   - `dev`: Unstable branch under development. Won't be merged to `main` unless it's finished.

- You can build images using this repository and `README.md`
   - You can also downlaod prebuilt images from Docker Hub
- All images do not use ENTRYPOINT options for ease of development
   - You can override any startup script when running a container
<br/>

## 2. Build Image

### 2.1 Prequisites

- Host must have [docker](https://docs.docker.com/engine/install/ubuntu/)installed (>=19.03)
- For GPU support, following conditions must be met.
   - Host must have Nvidia GPU with CUDA 11.2 support
   - Host must have Nvidia GPU Driver installed (=>418.71.07, >=Kepler)
   - Host must have [nvidia-docker-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed
      - Use `nvidia-docker2`
<br/>

- Install Nvidia GPU Driver 470 by following command:
   - You don't need to do this on WSL2.
   - For WSL, install Nvidia Drivers on Windows instead

```shell
$ sudo apt install nvidia-driver-470
```
<br/>

- Host must have at least 20 GB of smpty storage (Recommended)
- Host must be X86 device
<br/>

### 2.2 Build Command

#### 2.2.1 Build Command Basics

- This repository uses Docker Buildkit for faster, efficient build
   - Attempt to build image without `DOCKER_BUILDKIT` will result in error
- Basic form of command for building an image is as following:

```shell
$ DOCKER_BUILDKIT=1 docker build --no-cache --build-arg <ARG>=<VAR> -t <IMAGE_NAME>:<TAG> -f <TARGET_DOCKERFILE> .
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
<br/>

#### 2.2.2 Setting `ARG` values

- Three following `ARG` values must be set to start build process:
   - `BASEIMAGE`: Base image to build an image
      - For `base` tag, recommend to use `ubuntu:20.04` or `nvidia/cuda`
      - For case of `airsim` or `gazebo` tag, use prebuilt base image name
   - `BASETAG`: Base image's tag to build an image
      - Depends on type and image you want to build
   - `PROCVER`: Whether a image is GPU version of CPU version
      - Must be one from `gpu` or `cpu`
      - Value other than them will provoke a build error.
<br/>

#### 2.2.3 Build Steps and Example

- `gazebo` 태그 이미지에 추가적으로 덧붙는 `novnc`의 경우 해당 경로의 `README.md 참고`

## 컨테이너 생성

- 컨테이너 생성 전, 다음 명령어를 호스트에서 실행

```shell
$ xhost + local:root
```

- 이후 다음 명령어를 통해 컨테이너를 생성

```bash
$ docker run --gpus all -itd --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --net=host --name <container_name> \
    acsldocker/simbase:<version>   
```

- 실행 직후 QGroundControl이 시작되며, QGC 종료시 컨테이너가 종료됨
- `docker exec -it <container_name> bash`를 이용해 컨테이너 터미널 작업 수행
- gzserver-gzclient 분리를 원할 시 `-e GAZEBO_MASTER_URI http://$(hostname  -I | cut -f1 -d' '):11345` 를 추가
   - gzserver 와 gzclient는 같은 내부망에 접속되어 있어야 함
   - WSL2 등의 환경에서는 $(hostname  -I | cut -f1 -d' ') 명령어 오류 발생 가능. 오류시 사설 IP를 직접 입력
- `-v /HOST_DIR/PX4-Autoiplot:/root/PX4-Autopilot`과 같이 Host PC의 PX4 폴더 마운트 가능
   - 이 경우 수정된 PX4 메시지를 사용자가 `CMakeLists.txt`, `uorb_rtps_message_ids.yaml`, 메시지 파일에 반영해줘야 함에 유의

## 시뮬레이션 실행

- QGroundControl은 다음 명령어를 통해 수동으로 실행 가능

```bash
$ su -c "/home/user/QGroundControl.AppImage" user
```

- PX4 SITL 실행은 다음 명령어를 통해 가능
- `gazebo_iris__baylands`는 사용 모델 및 world에 따라 변경 가능

```bash
$ make -C /root/PX4-Autopilot px4_sitl_rtps gazebo_iris__baylands
``` 

## 파일 및 폴더 설명

<details>
<summary><strong>PX4-Autopilot</strong></summary>

- 수정되거나 추가된 PX4 메시지와 그에 맞게 수정된 `CMakeLists.txt`, `uorb_rtps_message_ids.yaml`
- gazebo SITL에 대체해 사용할 수정 모델, `iris.sdf`

</details>


<details>
<summary><strong>ROS2-node</strong></summary>

- 강화학습 기반 제어와 Gazebo 센서 데이터 수신을 위한 ROS2 패키지

</details>


<details>
<summary><strong>models</strong></summary>

- gazebo SITL에 사용되는 iris 및 world 모델

</details>


<details>
<summary><strong>px4_ros_com_ros2</strong></summary>

- PX4 ROS 통신 패키지 및 Offboard Control 노드

</details>


<details>
<summary><strong>scrips</strong></summary>

- docker 이미지 빌드에 필요한 기타 파일 및 `entrypoint.sh` 스크립트
- **docker-clean**
   - ubuntu:focal 이미지 내 `/etc/apt/apt.conf.d/docker-clean`의 수정본
   - `APT::Install-Recommends "0"` 구문을 추가함으로써 모든 `apt install`에 `--no-install-recommends` 옵션을 부여
   - 첫 두 줄을 주석 처리하고 `APT::Keep-Downloaded-Packages=true`의 주석을 해제하면 `/var/cache/apt/archives/`에 내려받은 패키지 파일을 유지
- **sources.list**
   - 기반 이미지 내 `/etc/apt/sources.list`의 수정본
   - 패키지를 다운로드할 서버를 선택하는 파일, 기본값은 `archive.ubuntu.com`
   - `archive.ubuntu.com`를 국내 미러 서버인 `mirror.kakao.com`로 대체해 다운로드 속도 향상

</details>


<details>
<summary><strong>.gitignore</strong></summary>

- git 사용 시 특정 명칭의 파일 및 폴더가 포함되는 것을 방지
- `.autosave` 파일과 `build`, `install`, `log` 폴데를 제외
</details>

## 문제해결

<details>
<summary><strong>1. Tensorflow numa node 문제</strong></summary>

- Tensorflow 라이브러리 import 시 gpu는 불러와지나 다음과 같은 오류가 표시될 수 있음

```shell
successful NUMA node read from SysFS had negative value (-1), but there must be at least one NUMA node, so returning NUMA node zero
```

- 근본적인 해결책은 아니지만 Shell 명령어를 통한 해결책이 존재
- 먼저 Host 환경에서 다음 명령어를 실행해 VGA 번호를 확인 (lspci가 없을 시 `sudo apt instal pciutils`로 설치)

```bash
$ lspci | grep -i nvidia
xx:xx.a VGA compatible controller: NVIDIA Corporation Device 2482 (rev a1)
xx:xx.b Audio device: NVIDIA Corporation Device 228b (rev a1)
```

- `ls /sys/bus/pci/devices` 명령어를 사용하면 장치의 목록을 확인 가능
- 앞서 확인한 번호가 접두번호 뒤에 붙은 것을 확인 가능 (Ex. `0000:xx:xx.a`)
- 기본값은 -1이며 다음 명령어를 실행해 `numa_node` 값을 수정하고 확인

```bash
$ echo 0 | sudo tee -a /sys/bus/pci/devices/0000\:xx\:xx.a/numa_node
$ cat /sys/bus/pci/devices/0000\:xx\:xx.a/numa_node
0
```

- 이 값은 재부팅 이후 변경되기 때문에 재부팅 시마다 적용하기 위해서는 `crontab`을 이용
- 임시 방편이기 때문에 실제로는 `nvidia-smi topo -m`를 실행해 나오는 `NUMA Affinity`를 참고하는 것이 좋다

```bash
sudo crontab -e
# Crontab 실행 이후 기본 편집기 설정, 다음 구문을 입력해준다
# 편집기 재설정: sudo select-editor
@reboot (echo 0 | tee -a "/sys/bus/pci/devices/<PCI_ID>/numa_node")
```
