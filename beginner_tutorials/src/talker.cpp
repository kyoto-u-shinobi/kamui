#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GridCells.h"
#include "math.h"
#include "dynamixel_msgs/JointState.h"
#include "thermo_blob/blob.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include <tf/transform_datatypes.h>

nav_msgs::GridCells cells;
int count = 1;
/*
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
		cells.cell_width = 0.05;
		cells.cell_height = 0.05;
		cells.cells.resize(count);
		cells.cells[count-1].x = 0.01*count;
		cells.cells[count-1].y = 0.0;
		cells.cells[count-1].z = 0.0;5
}*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");	// 第３引数がnode名

	ros::NodeHandle n;	// nodeへのハンドラ

	// chatterという名前のtopicにstd_msgs::Stringという型のメッセージを送る
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher pub_state = n.advertise<std_msgs::Int16>("state", 1000);
	ros::Publisher pub_cmdvel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Publisher pub_range = n.advertise<sensor_msgs::Range>("range", 1000);
	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("map", 1000);
	ros::Publisher pub_cells = n.advertise<nav_msgs::GridCells>("cells", 1000);
	ros::Publisher pub_dyna = n.advertise<dynamixel_msgs::JointState>("tilt_controller/state", 1000);
	ros::Publisher pub_blob = n.advertise<thermo_blob::blob>("ThermoBlob", 1000);	
	ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose>("frontier_goal", 1000);

	//ros::Subscriber sub_range = n.subscribe("range", 1000, rangeCallback);

	ros::Rate loop_rate(100);	// ループ頻度を設定(10 Hz)

	int count;
	//int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;
		geometry_msgs::Twist cmd;
		std_msgs::Int16 state;
		sensor_msgs::Range range;
		nav_msgs::OccupancyGrid map;
		dynamixel_msgs::JointState dyna;
		thermo_blob::blob blob;
		geometry_msgs::Pose goal_pose;
		
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		cmd.linear.x = count/1000.0;
		cmd.linear.y = count/1000.0;
		cmd.linear.z = count/1000.0;

		cmd.angular.x = -count/1000.0;
		cmd.angular.y = -count/1000.0;
		cmd.angular.z = -count/1000.0;

		state.data = 0;
		
		range.header.frame_id = "/thermocam_link";
		range.field_of_view = 45*M_PI/180;
		range.min_range = 0.5;
		range.max_range = 20.0;
		range.range = 0.5;

		map.header.frame_id = "/odom";
		map.header.stamp = ros::Time::now();

		map.info.map_load_time = ros::Time::now();
 		map.info.width = 500;
		map.info.height = 1000;
		map.info.resolution = 0.05;
		map.info.origin.position.x = 0.0;
		map.info.origin.position.y = 0.0;
		map.info.origin.position.z = 0.0;
		map.info.origin.orientation.x = 0.0;
		map.info.origin.orientation.y = 0.0;
		map.info.origin.orientation.z = 0.0;
		map.info.origin.orientation.w = 1.0;

		map.data.resize(map.info.width * map.info.height);
		for(int i = 0; i < map.info.width * map.info.height; i++) map.data[i] = 100;
		map.data[10] = -1;
		map.data[9] = 100;


		cells.header.frame_id = "/odom";
		cells.header.stamp = ros::Time::now();
		cells.cell_width = 1;
		cells.cell_height = 1;
		cells.cells[0].x = 0;
		cells.cells[0].y = 0;
		cells.cells[0].x = 0;

		dyna.current_pos = M_PI/2*sin(0.01*count);
		
		double target_yaw = -3*M_PI/4;
		goal_pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

		//chatter_pub.publish(msg);
		//pub_cmdvel.publish(cmd);
		//pub_state.publish(state);
		//pub_range.publish(range);
		//pub_map.publish(map);
		pub_cells.publish(cells);
		//pub_dyna.publish(dyna);
		//pub_blob.publish(blob);
		//goal_pub.publish(goal_pose);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
