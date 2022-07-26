## 1. Actuator Geometry

### 1.1 Target Files

- `src/lib/mixer/<MIXER_TYPER>/CMakeLists.txt`
- `src/lib/mixer/<MIXER_TYPER>/geometry/<NEW_GEOMETRY>.toml`

## 2. Mixer Definition

### 2.1 Target Files

- `ROMFS/px4fmu_common/mixers/CMakeLists.txt`
- `ROMFS/px4fmu_common/mixers/<MIXER_NAME>.main.mix`

## 3. Airframe Definition

### 3.1 Target Files

- `ROMFS/px4fmu_common/airframes/<VEHICLEE_NAME>`
- `ROMFS/px4fmu_common/airframes/CMakeLists.txt`

## 4. Add to SITL target

### 4.1 Target File

- `platforms/posix/cmake/sitl_target.cmake`

## 5. (Optional) Add Model to Gazebo

### 5.1 Target Directory

- `Tools/sitl_gazebo/models`
- `/root/.gazebo/models`

## Reference

- [Mixing and Actuators](https://docs.px4.io/v1.12/en/concept/mixing.html)
- [Multicopter Geometry Files](https://docs.px4.io/v1.12/en/concept/geometry_files.html)
- [Simulation](https://docs.px4.io/v1.12/en/simulation/)
- [Gazebo Simulation](https://docs.px4.io/v1.12/en/simulation/gazebo.html)