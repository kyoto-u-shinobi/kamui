#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"
#include "dynamixel_msgs/JointState.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

nav_msgs::OccupancyGrid map;
int count = 1;
double field_of_view, range;
double camera_yaw;	// カメラ角度 [rad]

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{                      
	camera_yaw = msg->current_pos;
}

/*
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	

	map.header.frame_id = msg->header.frame_id;
	map.header.stamp = msg->header.stamp;

	map.info.map_load_time = msg->info.map_load_time;
	map.info.width = msg->info.width;
	map.info.height = msg->info.height;
	map.info.resolution = msg->info.resolution;
	map.info.origin.position.x = msg->info.origin.position.x;
	map.info.origin.position.y = msg->info.origin.position.y;
	map.info.origin.position.z = msg->info.origin.position.z;
	map.info.origin.orientation.x = msg->info.origin.orientation.x;
	map.info.origin.orientation.y = msg->info.origin.orientation.y;
	map.info.origin.orientation.z = msg->info.origin.orientation.z;
	map.info.origin.orientation.w = msg->info.origin.orientation.w;

	//ROS_INFO("%d %d", map.info.width, map.info.height);//debug

	map.data.resize(map.info.width * map.info.height);
	for(int i = 0; i < map.info.width * map.info.height; i++)
	{	
		map.data[i] = msg->data[i];
	}
}
*/
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	double xi, yi;
	field_of_view = msg->field_of_view;
	range = msg->range;

	tf::TransformListener& listener;

	geometry_msgs::PointStamped pt0, pt1, pt2;
	geometry_msgs::PointStamped pt0_tf, pt1_tf, pt2_tf;
	ros::Time cur_time = ros::Time::now();
	
	pt0.header.frame_id = "camera_link";
	pt1.header.frame_id = "camera_link";
	pt2.header.frame_id = "camera_link";

	pt0.header.stamp = cur_time;
	pt1.header.stamp = cur_time;
	pt2.header.stamp = cur_time;

	pt0.point.x = 0;
	pt0.point.y = 0;
	pt0.point.z = 0;
	pt1.point.x = range;
	pt1.point.y = -range*tan(field_of_view/2);
	pt1.point.z = 0;
	pt2.point.x = range;
	pt2.point.y = range*tan(field_of_view/2);
	pt2.point.z = 0;

	try
	{
    	listener.transformPoint("map", pt0, pt0_tf);
	}
	catch(tf::TransformException& ex)
	{
	    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
	}

	map.header.frame_id = "/map";
	map.header.stamp = cur_time;

	map.info.map_load_time = cur_time;
	map.info.resolution = 0.05;
 	map.info.width = (unsigned int)(range/map.info.resolution);
	map.info.height = (unsigned int)(range*tan(field_of_view/2)*2/map.info.resolution);	
	map.info.origin.position.x = 0.0;
	map.info.origin.position.y = -range*tan(field_of_view/2);
	map.info.origin.position.z = 0.0;
	map.info.origin.orientation.x = 0.0;
	map.info.origin.orientation.y = 0.0;
	map.info.origin.orientation.z = 0.0;
	map.info.origin.orientation.w = 1.0;

	map.data.resize(map.info.width * map.info.height);
	for(int i = 0; i < map.info.width * map.info.height; i++)
	{
		xi = map.info.origin.position.x + (i%map.info.width)*map.info.resolution + map.info.resolution/2;
		yi = map.info.origin.position.y + (i/map.info.width)*map.info.resolution + map.info.resolution/2;

		if(xi < range && xi > 0 && fabs(yi/xi) < tan(field_of_view/2))
		{
			map.data[i] = 0;
		}
		else
		{
			map.data[i] = -1;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nav_map_test");	// 第３引数がnode名

	ros::NodeHandle n;	// nodeへのハンドラ

	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("nav_map", 1000);

	ros::Subscriber sub_range = n.subscribe("range", 1000, rangeCallback);
	//ros::Subscriber sub_map = n.subscribe("map", 1000, mapCallback);
	ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);

	ros::Rate loop_rate(100);	// ループ頻度を設定(10 Hz)

	//int count = 0;
	while (ros::ok())
	{		
		pub_map.publish(map);
		
		//ROS_INFO("%f %f", range, field_of_view);//debug

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
