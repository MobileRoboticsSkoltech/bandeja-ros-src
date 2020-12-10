#include "ros/ros.h"
#include "mcu_interface/AlignMcuCamPhase.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "align_mcu_cam_phase_client");
  if (argc != 2)
  {
    ROS_INFO("usage: align_mcu_cam_phase_client X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mcu_interface::AlignMcuCamPhase>("align_mcu_cam_phase");
  mcu_interface::AlignMcuCamPhase srv;
  srv.request.a = atoll(argv[1]);
  //srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Response: %s", srv.response.response.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service align_mcu_cam_phase");
    return 1;
  }

  return 0;
}