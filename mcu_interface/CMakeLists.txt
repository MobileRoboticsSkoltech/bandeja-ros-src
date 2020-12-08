cmake_minimum_required(VERSION 2.8.3)
project(mcu_interface) #TODO same name as in package.xml
set (CMAKE_CXX_STANDARD 11)

# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    serial
    dynamic_reconfigure
)

# add Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/mcu_interface.cfg
)



catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mcu src/mcu.cpp)

# Dynamic reconfigure make sure configure headers are built before any node using them
add_dependencies(mcu ${PROJECT_NAME}_gencfg)

target_link_libraries(mcu ${catkin_LIBRARIES})
