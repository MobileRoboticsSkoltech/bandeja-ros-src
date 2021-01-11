#include "ros/ros.h"
#include "mcu_interface/PublishS10Timestamp.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_s10_timestamp_client");
  if (argc != 3)
  {
    ROS_INFO("usage: publish_s10_timestamp_client mcu_timestamp s10_timestamp");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mcu_interface::PublishS10Timestamp>("publish_s10_timestamp");
  mcu_interface::PublishS10Timestamp srv;
  srv.request.mcu_timestamp = std::stod(argv[1]);
  srv.request.s10_timestamp = std::stod(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Response: %s", srv.response.response.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service publish_s10_timestamp");
    return 1;
  }

  return 0;
}