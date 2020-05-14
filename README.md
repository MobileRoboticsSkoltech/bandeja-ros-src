# Samsung Avatar Dataset

## Sensors utilized:
<table>
<tr>
    <td>Lidar</td>
    <td>Velodyne VLP-16</td>
</tr>
<tr>
    <td>Visual cameras</td>
    <td>Basler</td>
</tr>
<tr>
    <td>Depth camera</td>
    <td>Kinect Azure</td>
</tr>
<tr>
    <td>MCU-platform</td>
    <td>STM32F4DISCOVERY</td>
</tr>
<tr>
    <td>IMU</td>
    <td>MPU9150</td>
</tr>
<tr>
    <td>Phone</td>
    <td>Samsung S10E</td>
</tr>
</table>

## Prerequizites
Before building the packages by `catkin_make` command the following software must be installed:
- for Basler visual cameras __pylon 5.1.0 Camera Software Suite Linux x86 (64 bit) - Debian Installer Package__ from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/pylon-5-1-0-linux-x86-64-bit-debian/)  
- for Azure camera __Azure Kinect Sensor SDK__ from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions)  

## Full roslaunch
The following command is used for launching all of the sensors:  

`roslaunch data_collection datacollection.launch`

## Velodyne LIDAR
Patched package is used for hardware time synchronization of lidar

## Basler Visual Cameras
## Azure Depth Camera


## MCU-IMU node
MCU-IMU node is receiving data from hardware platform via virtual serial port and publishing the following topics:
`/imu` - IMU measurements  
`/imu_temp` - IMU measured temperature  
`/lidar_ts` - platform local timestamp of triggered pulses for hardware lidar synchronization  
`/cameras_ts` - platform local timestamp of triggered pulses for hardware visual and depth cameras synchronization  

## Serial
`serial` is a C++ library for seial interface with MCU ([repo](https://github.com/wjwwood/serial.git)).

## Samsung S10E Phone
