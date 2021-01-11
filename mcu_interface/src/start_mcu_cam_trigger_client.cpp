#include "ros/ros.h"
#include "mcu_interface/StartMcuCamTriggering.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_mcu_cam_triggering_client");
  if (argc != 2)
  {
    ROS_INFO("usage: start_mcu_cam_triggering_client X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mcu_interface::StartMcuCamTriggering>("start_mcu_cam_triggering");
  mcu_interface::StartMcuCamTriggering srv;
  srv.request.a = std::stod(argv[1]);
  //srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Response: %s", srv.response.response.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service start_mcu_cam_triggering");
    return 1;
  }

  return 0;
}