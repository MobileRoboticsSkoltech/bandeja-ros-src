#include "ros/ros.h"
#include "mcu_interface/PublishS10ToMcuOffset.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_s10_to_mcu_offset_client");
  if (argc != 2)
  {
    ROS_INFO("usage: publish_s10_to_mcu_offset_client offset\n(offset = s10_timestamp - mcu_timestamp)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mcu_interface::PublishS10ToMcuOffset>("publish_s10_to_mcu_offset");
  mcu_interface::PublishS10ToMcuOffset srv;
  srv.request.offset = std::stod(argv[1]);
  //srv.request.s10_timestamp = std::stod(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Response: %s", srv.response.response.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service publish_s10_to_mcu_offset");
    return 1;
  }

  return 0;
}