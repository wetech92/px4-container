# A4 VAI Itegrated Training Environment Quick Start Manual

## 1 Introduction

- This document suggests minimum and straightforward procedures to run A4-VAI integrated training environment simulator
- This document target **generic Debian-based X64 Linux systems**
    - Test was done on **Ubuntu 22.04.03 Jammy Jellyfish**
- This document is a quick start guide. Which means, there is no extra explanations for an action
    - In order to understand each process, please refer to other documents present in this repository
- **With good network connection, whole process will take about less than 20 minutes**
    - If you want the record, this will take about less than 10 minutes.

## 2 Setups

### 2.1 docker-ce

```shell
sudo apt-get remove -y docker docker-engine docker.io containerd runc
sudo apt install -y curl
curl -fsSL https://get.docker.com -o get-docker.sh
sh ./get-docker.sh
sudo usermod -aG docker $USER
sudo reboot -h now
```

### 2.2 docker-compose

```shell
sudo curl -L "https://github.com/docker/compose/releases/download/v2.11.2/docker-compose-linux-x86_64" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose
```

### 2.3 NVIDIA driver

```shell
sudo ubuntu-drivers autoinstall
```

### 2.4 NVIDIA container tookit

```shell
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```shell
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 2.5 Run A4-VAI ITE (Gazebo)

```shell
docker pull kestr3l/px4:gazebo-gpu-0.0.3
docker pull kestr3l/qgc-app:nobg-4.0.0
```

```shell
mkdir ~/gitcodes
git clone https://github.com/kestr31/px4-container.git -b dev ~/gitcodes
docker-compose up -f ~/gitcodes/px4-container/Gazebo/docker-compose.yml
```

### 2.6 Kill A4-VAI ITE

- `Ctrl + C`

```shell
docker-compose down -f ~/gitcodes/px4-container/Gazebo/docker-compose.yml
```

## 3 Reference

1. [Install Docker Engine on Ubuntu - Install using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2. [[Docker-Compose] 도커 컴포즈 설치 및 사용](https://soyoung-new-challenge.tistory.com/73)
3. [docker/compose: Define and run multi-container applications with Docker](https://github.com/docker/compose)
4. [NVIDIA Driver Installation Quickstart Guide](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)
5. [NVIDIA CONTAINER TOOLKIT - Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)