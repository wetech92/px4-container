# A4-VAI ILE PX4 Container Image: Base

## 1. Build

### 1.1 CPU Only

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
--build-arg BASEIMAGE=ubuntu \
--build-arg BASETAG=20.04 \
--build-arg PROCVER=cpu \
-t <IMAGE_NAME>:base-cpu-<VERSION> \
-f Base/Dockerfile .
```
<br/>

### 1.2 With GPU Support

```shell
DOCKER_BUILDKIT=1 docker build --no-cache \
--build-arg BASEIMAGE=nvidia/cuda \
--build-arg BASETAG=11.3.1-cudnn8-devel-ubuntu20.04 \
--build-arg PROCVER=gpu \
-t <IMAGE_NAME>:base-gpu-<VERSION> \
-f Base/Dockerfile .
```
<br/>

- GPU-supported version includes `tensorflow-gpu 2.5.0` and `tf-agents 0.8.0`

> Based on user's need, CUDA & cudNN's version may vary.<br/>
Check [nvidia/cuda](https://hub.docker.com/r/nvidia/cuda) for base imags with different CUDA & cudNN versions.
<br/>

### 1.3 Build Process Block Diagram

- `build.order.drawio`