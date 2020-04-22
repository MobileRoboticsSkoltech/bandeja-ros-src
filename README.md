# Samsung Avatar Dataset
## Samsung S10E
TBD

## Velodyne LIDAR
TBD

## Basler Visual Cameras
- Install __pylon 5.1.0 Camera Software Suite Linux x86 (64 bit) - Debian Installer Package__ from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/pylon-5-1-0-linux-x86-64-bit-debian/)
- clone https://github.com/basler/pylon-ros-camera.git

start  
roslaunch pylon_camera pylon_camera_node.launch

do for hardware triggering 
```
rosservice call /pylon_camera_node/set_trigger_mode "data : true"
rosservice call /pylon_camera_node/set_trigger_source "value : 1"
rosservice call /pylon_camera_node/set_trigger_selector "value : 0"
```

## Azure Depth Camera
- Install __Azure Kinect Sensor SDK__ from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions)
- clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git

## MCU-IMU firmware
TBD
## Serial
clone https://github.com/wjwwood/serial.git

## Full roslaunch
TBD

