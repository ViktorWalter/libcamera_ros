# Libcamera ROS driver

https://github.com/ctu-mrs/libcamera_ros

## Description

### Libcamera
libcamera is a new software library aimed at supporting complex camera systems directly from the Linux operating system. It enables us to drive the Raspberry Pi camera system directly from open-source code running on Arm processors. The proprietary code running on the Broadcom GPU, to which users have no access, is almost completely bypassed.

libcamera presents a C++ API to applications. It works at the level of configuring the camera and then allowing an application to request image frames. These image buffers reside in system memory and can be passed directly to still image encoders (such as JPEG) or to video encoders (such as h.264). Ancillary functions such as encoding images or displaying them are beyond the purview of libcamera itself.

This package does the wrapping for ROS.

Check the configuration file to see available control parameters: [config.yaml](https://github.com/ctu-mrs/libcamera_ros/blob/main/config/param.yaml).

## Prequisites

The `libcamera` library is required to be installed and in present path (must be findable by CMake), installation instructions:

1. Enable installation from unstable PPA [README.md](https://github.com/ctu-mrs/mrs_uav_system?tab=readme-ov-file#native-installation).
2. install libcamera package `sudo apt install ros-noetic-libcamera` 

Deb packages are available for arm64 and amd64 architectures. The driver is suppose to be used on arm64 (RaspberryPi5). 
However, installing it on amd64 makes is easier to work with during development.

Tested with ROS Melodic, but should work with older/newer versions as well.

## Running

Basic launch file

```bash
roslaunch libcamera_ros libcamera_ros.launch 
```

More complex launch file using the parameters from config file
```bash
roslaunch libcamera_ros uav.launch 
```


## Acknowledgements

The code was inspired by the driver for ROS2: https://github.com/christianrauch/camera_ros.
