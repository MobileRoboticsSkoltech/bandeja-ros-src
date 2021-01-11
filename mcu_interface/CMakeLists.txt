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
    message_generation
)

# add Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/mcu_interface.cfg
)

add_service_files(
  FILES
  AlignMcuCamPhase.srv
  StartMcuCamTriggering.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mcu src/mcu.cpp)

# Dynamic reconfigure make sure configure headers are built before any node using them
add_dependencies(mcu ${PROJECT_NAME}_gencfg)

target_link_libraries(mcu ${catkin_LIBRARIES})


add_executable(align_mcu_cam_phase_client src/align_mcu_cam_phase_client.cpp)
target_link_libraries(align_mcu_cam_phase_client ${catkin_LIBRARIES})
add_dependencies(align_mcu_cam_phase_client mcu_interface_gencpp)

add_executable(start_mcu_cam_trigger_client src/start_mcu_cam_trigger_client.cpp)
target_link_libraries(start_mcu_cam_trigger_client ${catkin_LIBRARIES})
add_dependencies(start_mcu_cam_trigger_client mcu_interface_gencpp)
