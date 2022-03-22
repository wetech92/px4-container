# A4-VAI 통합 시뮬레이터 환경 PX4 컨테이너 이미지

## 사전 요구사항


- Host PC에 nvidia 드라이버가 설치되어 있을 것 (>=418.81.07, Kepler 아키텍처 이상)

``` bash
# nvidia 드라이버 버전을 지정해 설치 (작성 시점 기준 470)
$ sudo apt install nvidia-driver-470
```

- Host PC에 [docker](https://docs.docker.com/engine/install/ubuntu/)가 설치되어 있을 것 (>=19.03)
- Host PC에 [nvidia-docker-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)이 설치되어 있을 것

## 수동 빌드

- base, gazebo, airsim에 해당하는 빌드 명령어는 아래와 같다
  - 저장소 최상단에서 대상 폴더를 고려해 빌드를 수행한다
  - gazebo 및 airsim 폴더에 대한 이미지 빌드 시 base 계열 태그의 이미지가 필요하다
  - 태그명의 경우 gazebo-1.0.0과 같은 식으로 식별 가능하게 지을 것을 권장한다

```shell
docker build -t <image_name>:<tag> -f ./<target_folder>/Dockerfile .
```

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
