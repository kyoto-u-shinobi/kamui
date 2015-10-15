#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

void subs_cmdvel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO(" [linear]x: %f   y: %f   z: %f", msg->linear.x,  msg->linear.y,  msg->linear.x);
	ROS_INFO("[angular]x: %f   y: %f   z: %f", msg->angular.x, msg->angular.y, msg->angular.x);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "subs_cmdvel");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, subs_cmdvel_callback);

	ros::spin(); // ループ　メッセージが来るのを待つ

	return 0;
}
