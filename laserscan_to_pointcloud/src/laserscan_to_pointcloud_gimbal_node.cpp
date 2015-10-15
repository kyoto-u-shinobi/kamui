#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

#define STOCK_SIZE 40
#define Z_MIN   0.05    // [m]
#define Z_MAX   0.55    // [m]
#define R_MIN   0.3     // [m]
#define R_MAX   2.0     // [m]

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_, scan_pub_2_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "scan", 1000),
    laser_notifier_(laser_sub_,listener_, "/base_link", 1000)
  {
    laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.1));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/gimbal/cloud",1);
    scan_pub_2_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_in",1);

    scan_count = 0;
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    //ROS_INFO("test0 scan_count = %d", scan_count);//debug

    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud cloud_out;
    //sensor_msgs::PointCloud cloud_tmp;
    try
    {
        projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud2, listener_);
        //ROS_INFO("test01");//debug
        projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud[scan_count], listener_);
        //ROS_INFO("test02");//debug
    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR("%s", e.what());
        return;
    }

    //cloud[scan_count].clear();
    //cloud[scan_count] = cloud_tmp;
    cloud_out.header = cloud[scan_count].header;

    for(int i = 0; i < STOCK_SIZE; i++)
    {
        size_t size = cloud[i].points.size();
        //ROS_INFO("size = %d", (int)size);//debug
        geometry_msgs::Point32 points[size];
        sensor_msgs::ChannelFloat32 channels[size];
        int z_count = 0;
        //ROS_INFO("test1");//debug
        for(int j = 0; j < size; j++)
        {
            if(cloud[i].points[j].z > Z_MIN && cloud[i].points[j].z < Z_MAX && sqrt(pow(cloud[i].points[j].x, 2.0) + pow(cloud[i].points[j].y, 2.0)) > R_MIN && sqrt(pow(cloud[i].points[j].x, 2.0) + pow(cloud[i].points[j].y, 2.0)) < R_MAX)
            {
                //ROS_INFO("j = %d", j);//debug
                points[z_count].x = cloud[i].points[j].x;   //ROS_INFO("x");
                points[z_count].y = cloud[i].points[j].y;   //ROS_INFO("y");
                points[z_count].z = cloud[i].points[j].z;   //ROS_INFO("z");

                /*
                channels[z_count].name = "intensity";   ROS_INFO("name");//cloud[i].channels[j].name;
                ROS_INFO("values size = %d", (int)cloud[i].channels[j].values.size());//debug

                for(int l = 0; l < cloud[i].channels[j].values.size(); l++)
                {
                    channels[z_count].values[l] = 0;//cloud[i].channels[j].values[l];
                }
                */
                //ROS_INFO("test2");//debug
                z_count++;
            }
        }


        //ROS_INFO("z_count = %d", z_count);//debug

        size_t cur_size = cloud_out.points.size();
        //ROS_INFO("cur_size = %d", (int)cur_size);//debug
        cloud_out.points.resize(cur_size + z_count - 1);
        cloud_out.channels.resize(cur_size + z_count - 1);
        for(int k = 0; k < z_count - 1; k++)
        {
            cloud_out.points[cur_size + k].x = points[k].x;
            cloud_out.points[cur_size + k].y = points[k].y;
            cloud_out.points[cur_size + k].z = points[k].z;
            //ROS_INFO("test3 k = %d", k);//debug

            /*
            cloud_out.channels[cur_size + k].name = channels[k].name;
            for(int l = 0; l < channels[k].values.size(); l++)
            {
                cloud_out.channels[cur_size + k].values[l] = channels[k].values[l];
            }
            */
        }
        //ROS_INFO("test4");//debug
    }


    scan_pub_.publish(cloud_out);

    scan_count++;
    if(scan_count > STOCK_SIZE-1) scan_count = 0;

    scan_pub_2_.publish(cloud2);

  }

private:
  sensor_msgs::PointCloud cloud[STOCK_SIZE];
  int scan_count;

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "laserscan_to_pointcloud_gimbal_node");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
