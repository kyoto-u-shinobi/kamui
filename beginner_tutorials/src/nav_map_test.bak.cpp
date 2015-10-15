#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"
#include <tf/transform_listener.h>

nav_msgs::OccupancyGrid map;
int count = 1;
double field_of_view, range;

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

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	double xi, yi;

	tf::TransformListener listener;

	geometry_msgs::PoseStamped origin, origin_tf;

	origin.pose = map.info.origin;

	field_of_view = msg->field_of_view;
	range = msg->range;

	listener.transformPose("camera_link", ros::Time(0), origin, "map", origin_tf);

	for(int i = 0; i < map.info.width * map.info.height; i++)
	{
		xi = map.info.origin.position.x + (i%map.info.width)*map.info.resolution + map.info.resolution/2;
		yi = map.info.origin.position.y + (i/map.info.width)*map.info.resolution + map.info.resolution/2;

		if(xi < range && xi > 0 && fabs(yi/xi) < tan(field_of_view/2))
		{
			
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
	ros::Subscriber sub_map = n.subscribe("map", 1000, mapCallback);

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
