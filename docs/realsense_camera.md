# Realsense Camera

- Add apt key
```shell
sudo apt-get update # && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

- Add apt repository
```shell
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

## Install SDK2
- Install SDK2
```shell
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

- Install dev and debug tools (optional)
```shell
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

- Test SDK2
```shell
realsense-viewer 
```

## Install ROS version
```shell
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install ros-$ROS_DISTRO-realsense2-description
```

## Install rgbd-launch
- Install
```shell
sudo apt-get install ros-noetic-rgbd-launch
```

- Test
```shell
roslaunch realsense2_camera demo_pointcloud.launch 
```

## Errors
- The ddynamic library that comes with ros-noetic may cause **process died**.
```shell
sudo apt-get remove ros-noetic-ddynamic-reconfigure
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd ..
catkin_make
catkin_make install
```

## References
- CSDN: https://blog.csdn.net/wanghq2013/article/details/123325671
- CSDN: https://blog.csdn.net/qq_46107892/article/details/131481038?spm=1001.2014.3001.5506
