# A4-VAI Integrated Training Environment PX4 Container Image

- **WARNING! This is a `dev` branch under development**
- **Build based on contents from this branch maybe unstable**

## What is this repository for?

- This is a repository containing files to build `kestr3l/px4` images
- Those images were built to implement simulator system for Integrated Training Environment (ITE)

## Image Tags

- There are three matjor types of images:
   - `kestr3l/px4:base`: Base image including dependcies to run PX4-Autopilot and ROS2-Galactic
      -Base image of `airsim` and `gazebo` image
   - `kestr3l/px4:airsim`: Image to run PX4 AirSim SITL
   - `kestr3l/px4:gazebo`: Image to run PX4 Gazebo SITL

- Minor tags of image are as following:
   - `gpu`: Image for Machine Learning, Reinforced Learning and etc.
      - Based on `nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04`
   - `cpu`: Image withut GPU support. Intended to be used on device with no GPU
      - Based on `ubuntu:20.04`

- Naming rule of image is as `kestr3l/px4:\<MAJOR\>-\<MINOR\>-\<VERSION\>`
  - Example: `kestr3l/px4:airsim-gpu-0.0.2`
   - Image for AirSim SITL with GPU support, version 0.0.2

## Branches and Miscellaneous

- Currently, we have two branches in this repository
   - `main`: Main, stable branch. Check tag and `CHANGELOG.md `for verision info 
   - `dev`: Unstable branch under development. Won't be merged to `main` unless it's finished.

- You can build images using this repository and `README.md`
   - You can also downlaod prebuilt images from [Docker Hub](https://hub.docker.com/r/kestr3l/px4)
- All images do not use ENTRYPOINT options for ease of development
   - You can override any startup script when running a container

## Guides For Using This Repository

- For list of guides available, please proceed to [A4-VAI ITE User Guide Documents](docs/README.md)
- Documents are located in `/docs` directory