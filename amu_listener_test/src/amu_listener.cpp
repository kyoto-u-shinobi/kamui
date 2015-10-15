#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  ROS_INFO("RPY = (%lf, %lf, %lf)[rad]", roll, pitch, yaw);
  ROS_INFO("dac = (%lf, %lf, %lf)[rad/sec]", msg->angular_velocity.x,  msg->angular_velocity.y,  msg->angular_velocity.z);
  ROS_INFO("acc = (%lf, %lf, %lf)[m/s^2]", msg->linear_acceleration.x,  msg->linear_acceleration.y,  msg->linear_acceleration.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amu_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/imu/data", 1, chatterCallback);

  ros::spin();

  return 0;
}
