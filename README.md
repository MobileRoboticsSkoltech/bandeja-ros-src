
The repository contains modified `Azure_Kinect_ROS_Driver` and our own `mcu_interface` ROS packages for synchronized along with an Android Smartphone data gathering.

## Sensors utilized:
<table>
    <tr> <td>Phone</td> <td>Samsung S10E</td> </tr>
    <tr> <td>Depth camera</td> <td>Kinect Azure</td> </tr>
    <tr> <td>MCU-platform</td> <td>STM32F4DISCOVERY</td> </tr>
    <tr> <td>IMU</td> <td>MPU9150</td> </tr>
</table>

## Prerequisites
- OS version: Ubuntu 18.04  
- ROS version: Melodic. Install Desktop-Full from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

- Before building the packages by `catkin_make` command the Azure camera __Azure Kinect Sensor SDK__ from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download#linux-installation-instructions) must be installed.
- After cloning this repo do clone 
    - [serial](https://github.com/wjwwood/serial)
    - [Azure_Kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver.git)
    - [dragandbot_common](https://github.com/dragandbot/dragandbot_common.git)
- To be able to use serial interface by MCU package, run  
`sudo usermod -a -G dialout $USER`
Then log out and log in.  

    
## Full roslaunch
The following command is used for launching all of the sensors:  

`roslaunch data_collection data_collection.launch`

## Azure Depth Camera
Our modification contains return to Azure Kinect DK internal timestamping procedure instead of ROS timestatmping that create low-precision and low-accurate time measurements.

## MCU-IMU node
MCU-IMU node is receiving data from hardware platform via virtual serial port and publishing the following topics:  
`/imu` - IMU measurements  
`/imu_temp` - IMU measured temperature  
`/cameras_ts` - platform local timestamps of triggered pulses for hardware visual and depth cameras synchronization  

## Serial
`serial` is a C++ library for seial interface with MCU ([repo](https://github.com/wjwwood/serial.git)).

## Smartphone
Samsung S10E is used for data gathering powered by our [OpenCamera-Sensors](https://github.com/MobileRoboticsSkoltech/OpenCamera-Sensors) app for time and frame synchronized image retrival.
