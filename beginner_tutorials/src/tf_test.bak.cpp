#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/Range.h"
#include "dynamixel_msgs/JointState.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"
#include "tf/message_filter.h"

nav_msgs::OccupancyGrid map;
double field_of_view, range;
geometry_msgs::PointStamped pt0_tf, pt1_tf, pt2_tf;

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	double xi, yi;
	field_of_view = msg->field_of_view;
	range = msg->range;
	
	ros::Time cur_time = ros::Time::now();

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

void transformPoint(const tf::TransformListener& listener){
	geometry_msgs::PointStamped pt0, pt1, pt2;
	ros::Time cur_time = ros::Time::now();
	
	pt0.header.frame_id = "camera_link";
	pt1.header.frame_id = "base_footprint";//"camera_link";
	pt2.header.frame_id = "base_footprint";//"camera_link";

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
		//listener.transformPoint("map", pt1, pt1_tf);
		//listener.transformPoint("map", pt2, pt2_tf);
		ROS_INFO("trying transform...");//debug
	}
	catch(tf::TransformException& ex)
	{
	    ROS_ERROR("Received an exception trying to transform a point from \"camera_link\" to \"map\": %s", ex.what());
	}
}



int main(int argc, char** argv){
	ros::init(argc, argv, "tf_test");
	ros::NodeHandle n;

	ros::Publisher pub_pts = n.advertise<geometry_msgs::PointStamped>("points", 1000);
	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("nav_map", 1000);

	ros::Subscriber sub_range = n.subscribe("range", 1000, rangeCallback);
	//ros::Subscriber sub_map = n.subscribe("map", 1000, mapCallback);
	//ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);

	tf::TransformListener listener(ros::Duration(10));

	//we'll transform a point once every second
	ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

	ros::Rate loop_rate(100);

	ros::spin();
	/*
	while (ros::ok())
	{
		//transformPoint(listener);
		boost::bind(&transformPoint, boost::ref(listener));
	
		pub_pts.publish(pt0_tf);

		ros::spinOnce();

		loop_rate.sleep();
	}
*/
}
