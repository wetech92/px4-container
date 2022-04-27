- `px4:gazebo-novnc` 태그의 컨테이너 이미지
  - `novnc`를 이용해 브라우저를 통한 작업 가능 (내부 포트 `6901`)

## 수동 빌드

- [Docker BuildKit](https://docs.docker.com/develop/develop-images/build_enhancements/)을 이용한 빌드 수행

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg ARG_HEADLESS_USER_NAME=user \
    -t acsldocker/px4:gazebo-nv-<version> .
```

- 예시
  - `acsldocker/px4:gazebo-0.0.1` 이미지를 가지고 있고 `px4` 경로에서 빌드 시
  - 빌드 결과 태그명을 `acsldocker/px4:gazebo-novnc-0.0.1`으로 설정하고자 할 시

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
    --build-arg ARG_HEADLESS_USER_NAME=user \
    -t acsldocker/px4:gazebo-novnc-0.0.1 \
    -f ./novnc/Dockerfile .
```

## 실행 및 접속

- 기본 실행 명령어
  - 필요에 따라 `gpus -all`, 볼륨 마운트 등 옵션을 추가 가능
  - 포트 번호는 5자리 이상을 권장, 사용자 필요에 따라 설정

```shell
docker run --privileged -d \
    -p <novnc_port>:6901 \
    acsldocker/px4:gazebo-novnc-<version>
```

- 예시
  - 외부 접속 포트를 `10000`으로 설정하고 GPU를 사용할 때
  - 사용 이미지가 `acsldocker/px4:gazebo-novnc-0.0.1`일 때

```shell
docker run --privileged -d --gpus all \
    -p 10000:6901 \
    acsldocker/px4:gazebo-novnc-0.0.1
```

- 접속
  - 브라우저를 통해 `http://<server_ip>:<novnc_port>/vnc.html`로 접속
  - 기본 비밀번호는 `headless`
    - VNC 비밀번호 변경은 빌드 시 `--build-arg ARG_VNC_PW=newpassword`와 같이 ARG override로 가능
    - 사용자 계정 비밀번호는 빌드 시 `--build-arg ARG_SUDO_PW=newpassword` 적용


## 문제 해결

1. novnc 최초 실행 시 xfce4 바탕화면 아이콘을 통한 Terminal 열리지 않음 문제
- 터미널 아이콘의 연결 명령어를 xfce4-terminal로 수정

## 출처 및 참고

1. [accetto/ubuntu-vnc-xfce-g3 Headless Ubuntu/Xfce containers with VNC/noVNC](https://github.com/accetto/ubuntu-vnc-xfce-g3)