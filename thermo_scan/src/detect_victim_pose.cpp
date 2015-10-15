#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "dynamixel_msgs/JointState.h"
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/Range.h"
#include "std_srvs/Empty.h"

#define	OCCUPIED	100		// 黒
#define OBSERVED	127		// 緑（rviz上での表示色）
#define UNSIGHT		-2		// 黄

#define R_MAX		10
#define R_STEP		2


nav_msgs::OccupancyGrid map0;
geometry_msgs::PoseArray victims0;
double camera_yaw;	// カメラ角度 [rad]
int flag = 0;
double range= 0.6;
int victim_count;
/*
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{                      
	range = msg->range;
}

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{                      
	camera_yaw = msg->current_pos;
}
*/
bool add_victim(std_srvs::Empty::Request  &req,
		 	    std_srvs::Empty::Response &res)
{
	double xr, yr, rot;
	double target_x, target_y, target_angle;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	nav_msgs::OccupancyGrid map;
	try
	{
		//listener.lookupTransform("/map", "/thermocam_link", ros::Time(0), transform);
		ros::Time now = ros::Time(0);
		listener.waitForTransform("/map", "/thermocam_link", now, ros::Duration(3.0));
		listener.lookupTransform("/map", "/thermocam_link", now, transform);
	}
    catch (tf::TransformException ex)
	{
  		ROS_ERROR("%s",ex.what());
	}
	rot = getYaw(transform.getRotation());

	target_angle = rot + /*camera_yaw + */M_PI;
	if(target_angle > M_PI)	target_angle -= 2*M_PI;
	else if(target_angle < -M_PI)	target_angle += 2*M_PI;


	map = map0;
	for(double rj=0; rj <= R_MAX; rj += map.info.resolution/R_STEP)
	{
		//ROS_INFO("rj=%f", rj);//debug
		unsigned int index;
		xr = rj * cos(/*camera_yaw + */rot) + transform.getOrigin().x() - map.info.origin.position.x;
		yr = rj * sin(/*camera_yaw + */rot) + transform.getOrigin().y() - map.info.origin.position.y;
		index = (unsigned int)(yr/map.info.resolution)*map.info.width + (unsigned int)(xr/map.info.resolution);
		if(index < map.data.size() && map.data[index] == OCCUPIED)
		{
			target_x = xr;
			target_y = yr;
			ROS_INFO("detect victim position: rj=%f", rj);//debug
			break;
		}
	}
	
	victim_count++;

	ROS_INFO("Add victim no.%d!", victim_count);

	victims0.header.frame_id = "/map"; 
	victims0.header.stamp = ros::Time::now();
	victims0.poses.resize(victim_count);
	victims0.poses[victim_count-1].position.x = target_x + map.info.origin.position.x;
	victims0.poses[victim_count-1].position.y = target_y + map.info.origin.position.y;
	victims0.poses[victim_count-1].orientation = tf::createQuaternionMsgFromYaw(target_angle);

	flag = 1;
	return true;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("mapCallback");//debug
	map0 = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detect_victim_pose_node");

	ros::NodeHandle n;	// nodeへのハンドラ

	ros::Publisher pub_vic = n.advertise<geometry_msgs::PoseArray>("victims", 1000);
	ros::Subscriber sub_map = n.subscribe("map", 10, mapCallback);
	ros::ServiceServer service = n.advertiseService("addVictim", add_victim);
	//ros::Subscriber sub_dyna = n.subscribe("tilt_controller/state", 1000, DynaCallback);
	//ros::Subscriber sub_range = n.subscribe("range", 1000, rangeCallback);

	ros::Rate loop_rate(10);

	geometry_msgs::PoseArray victims;

	victim_count = 0;

	while (ros::ok())
	{
		if(flag)
		{
			victims = victims0;
			pub_vic.publish(victims);
		}

		ros::spinOnce();

		loop_rate.sleep();
	}
}
