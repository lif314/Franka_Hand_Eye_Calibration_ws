# Franka_Hand_Eye_Calibration
Franka Hand Eye Calibration (Panda or FR3)

- Hardwares:
    - camera: RealSense d435i
    - OS: Ubuntu 20.04
    - Robot: Franka Research 3

## Install ros packages

```shell
cd src
```

- Install [vision_visp]
```shell
git clone -b neotic-devel https://github.com/lagadic/vision_visp.git
# OR
git clone -b neotic-devel git@github.com:lagadic/vision_visp.git

# remove unsed pkgs (optional)
cd vision_visp && rm -rf visp_tracker visp_auto_tracker

```

- Install [aruco_ros](https://github.com/pal-robotics/aruco_ros)
```shell
git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros
# OR
git clone -b neotic-devel git@github.com:pal-robotics/aruco_ros.git
```

- Install [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
```shell
git clone https://github.com/IFL-CAMP/easy_handeye
# OR
git clone git@github.com:IFL-CAMP/easy_handeye.git
```

- Install [panda_moveit_config](https://github.com/ros-planning/panda_moveit_config)
```shell
git clone -b neotic-devel https://github.com/ros-planning/panda_moveit_config.git
# OR
git clone -b noetic-devel git@github.com:ros-planning/panda_moveit_config.git
```

- Generate ArUco markers
    - https://chev.me/arucogen/

- make && build
```

```


## Calibration
```shell
source devel/setup.bash
```

- camera
```shell
roslaunch realsense2_camera rs_camera.launch 
```

- panda_realsense
```shell
roslaunch easy_handeye panda_realsense_eyeonbase.launch 
```

- easy panda
```shell
roslaunch easy_handeye easy_panda.launch
```


## References
- CSDN: https://blog.csdn.net/ZNC1998/article/details/132475019
- FR3Setup: https://fr3setup.readthedocs.io/en/latest/prerequisites/index.html