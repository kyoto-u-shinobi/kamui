#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"

#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "param_test");	// 第３引数がnode名

	ros::NodeHandle n;	// nodeへのハンドラ

	std_msgs::String msg;
	geometry_msgs::Twist cmd;
	std_msgs::Int16 state;
	std::string str, default_param;
	int num;
	float flt;
	double dbl;

	// chatterという名前のtopicにstd_msgs::Stringという型のメッセージを送る
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher pub_state = n.advertise<std_msgs::Int16>("state", 1000);
	ros::Publisher pub_cmdvel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Rate loop_rate(10);	// ループ頻度を設定(10 Hz)

	// Default value version
	ros::param::param<std::string>("default_param", default_param, "default_value");

	int count = 0;
	while (ros::ok())
	{
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		state.data = 0;

		//ROS_INFO("%s", msg.data.c_str());	// printfやcoutのようなもの
		//ROS_INFO(ROS_PACKAGE_NAME);

		ros::param::get("param1", str);
		ROS_INFO("param1: %s", str.c_str());

		ros::param::get("param2", num);
		ROS_INFO("param2: %d", num);

		ros::param::get("param3", flt);
		ROS_INFO("param3: %f", flt);

		ros::param::get("param4", dbl);
		ROS_INFO("param4: %f", dbl);

		ROS_INFO("%s", default_param.c_str());

		chatter_pub.publish(msg);
		pub_cmdvel.publish(cmd);
		pub_state.publish(state);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
