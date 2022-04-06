# CHANGELOG

## [0.0.1] - 2022-04-06
 
- 저장소 생성
- nvidia/cuda:11.2.2-cudnn8-devel-ubuntu20.04
  - Gazebo 11.9.0
 
### 추가사항

- PX4 컨테이너 생성
- `px4-base` 컨테이너를 기반으로 AirSim SITL과 Gazebo SITL을 위한 컨테이너로 분화
  - Gazebo의 경우 iris 수정본(카메라 및 LIDAR 장착)과 baylands가 기본값
 
### 수정사항