# List of Common Convenience Scipts and Examples

- A4-VAI Itegrated Training Environment includes various convenience scripts for ease of development & testing
  - Some of them are run by `entrypoint.sh` during container initialization process
- This document is a list of shell/python scripts included in A4-VAI ITE containers
- Following scripts are located in `/root/tools`. Scripts that are useless after build process are not on this list

## Common (Both Gazebo and AirSim)

### `buildRos2Pkg.sh`

- Build all pkgs defined in ROS workspace `/root/ros_ws` and add them to `.bashrc` or `.zshrc`
- Run example:

```shell
/root/tools/buildRos2Pkg.sh
```

### `initRepo.sh`

- Add ROS2 repository and get new list of packages by `apt update`
- This scrips exists since initial docker container contains no any data about lists of available packages
  - Manually adding a ROS package repository is very tedious and bothersome.
- Run example:

```shell
/root/tools/initRepo.sh
```

### `rebuildPX4ros.sh`

- Clear all uRTPS messages and `urtps_bridge_topics.yaml` and replace them by newly generated ones
- This script can be used to 'clean-up' uRTPS message definitions when something went wrong
- Run example:

```
/root/tools/rebuildPX4ros.sh
```