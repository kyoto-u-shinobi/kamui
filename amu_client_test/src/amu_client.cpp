#include "ros/ros.h"
#include "amu_client_test/amu_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amu_client_test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<amu_client_test::amu_control>("amu_control");
  
  ros::Duration(1.0).sleep();
  
  amu_client_test::amu_control srv;
  srv.request.command = 1;
  client.call(srv);
  ROS_INFO("send command");

  return 0;
}
