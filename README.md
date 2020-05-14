# Samsung Avatar Dataset

Before building the packages by `catkin_make` command the following software must be installed:

- for Basler visual cameras __pylon 5.1.0 Camera Software Suite Linux x86 (64 bit) - Debian Installer Package__ from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/pylon-5-1-0-linux-x86-64-bit-debian/)  
- for Azure camera __Azure Kinect Sensor SDK__ from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions)  

Sensors used:
## Basler Visual Cameras
## Azure Depth Camera
## Velodyne LIDAR
Patched package is used

## MCU-IMU firmware

## Serial
`serial` is a C++ library for seial interface with MCU ([repo](https://github.com/wjwwood/serial.git)).

## Full roslaunch
Full roslaunch for launching all of the sensors 
`roslaunch data_collection datacollection.launch`
