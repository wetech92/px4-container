# Setting Up an Environment

## 1 Introduction

- This document suggests mandatory or recommended procedures to configure an environment to run A4-VAI integrated training environment simulator
- This document target **generic Debian-based X64 Linux systems**
    - Test was done on **Ubuntu 22.04.03 Jammy Jellyfish**

### 1.1 Required Components

- Following components **MUST** be installed to run ITE
  - docker-ce
  - docker-compose
  - NVIDIA driver
  - NVIDIA container tookit

### 1.2 Recommended Components

- Following components will greatly improve your productivity
  - Oh-My-ZSH and plugins
  - Visual Studio Code
  - Powerline fonts

## 2 Setting Up Mandatory Requirements

- Before performing setup, it is recommended to perform following command:

```shell
sudo apt update && sudo apt upgrade -y
```

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

## 3 Setting Up Recommended Requirements

### 3.1 Visual Studio Code

```shell
sudo apt update && sudo apt install -y code
```

- Then, run code and press `Ctrl + Shift + X`. Install `Remote - SSH` extension and `Dev Containers` extension

### 3.2 Oh-My-ZSH and plugins

- Install ZSH and Oh-My-Zsh by following commands:

```shell
sudo apt update && sudo apt install -y zsh curl bat
chsh -s $(which zsh)
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```

- Clone plugins for application

```shell
git clone https://github.com/zsh-users/zsh-autosuggestions ${HOME}/.oh-my-zsh/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting ${HOME}/.oh-my-zsh/plugins/zsh-syntax-highlighting
gedit ${HOME}/.zshrc
```

- On opened text editor, edit following parts as shown and save:
  - `alias cat='batcat --paging=never'` is a added line

```zshrc
...
ZSH_THEME="agnoster"
...
plugins=(git
	 docker
	 zsh-autosuggestions
	 zsh-syntax-highlighting
	)
...
alias cat='batcat --paging=never'
```

- Run following command to apply change
   - Crash in font is expected. This is solved by installing powerline fonts

```shell
source ~/.zshrc
sudo reboot -h now
```

### 3.3 Powerline fonts

- To install it system-wide, install powerline fonts by following command

```
sudo apt update && sudo apt-get install -y fonts-powerline
sudo reboot -h now
```

- Application in Visual Studio Code should be done separately
    - Open settings by `Ctrl + ,`. Search `Font Family` and add `'Ubuntu Mono derivative Powerline'` on front of the list

## 4 Reference

1. [Install Docker Engine on Ubuntu - Install using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)
2. [[Docker-Compose] 도커 컴포즈 설치 및 사용](https://soyoung-new-challenge.tistory.com/73)
3. [docker/compose: Define and run multi-container applications with Docker](https://github.com/docker/compose)
4. [NVIDIA Driver Installation Quickstart Guide](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)
5. [NVIDIA CONTAINER TOOLKIT - Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
6. [zsh-users/zsh-autosuggestions: Fish-like fast/unobtrusive autosuggestions for zsh](https://github.com/zsh-users/zsh-autosuggestions)
7. [zsh-users/zsh-syntax-highlighting: Fish shell-like syntax highlighting for Zsh](https://github.com/zsh-users/zsh-syntax-highlighting)
8. [sharkdp/bat: A cat(1) clone with syntax highlighting and Git integration.](https://github.com/sharkdp/bat)
9. [powerline/fonts: Patched fonts for Powerline users.](https://github.com/powerline/fonts)