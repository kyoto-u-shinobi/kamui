#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

ros::Publisher pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    sensor_msgs::Image pub_image;

    // 最新のMap画像をセット
    pub_image.header = msg->header;
    pub_image.height = msg->width;
    pub_image.width = msg->height;
    pub_image.encoding = msg->encoding;
    pub_image.step = pub_image.width*3;
    pub_image.data.resize(msg->data.size());

    for(int i = 0; i < msg->height; i++)
    {
        for(int j = 0; j < msg->width; j++)
        {
            pub_image.data[3*j*msg->height+3*(msg->height-i)] = msg->data[3*i*msg->width+3*j];
            pub_image.data[3*j*msg->height+3*(msg->height-i)+1] = msg->data[3*i*msg->width+3*j+1];
            pub_image.data[3*j*msg->height+3*(msg->height-i)+2] = msg->data[3*i*msg->width+3*j+2];
        }
    }

    pub.publish(pub_image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_thermo_image_node");

    ros::NodeHandle n;
    pub = n.advertise<sensor_msgs::Image>("rotated_thermal_image_view", 10);
    ros::Subscriber sub = n.subscribe("thermal_image_view", 10, imageCallback);

    ros::spin();

}
