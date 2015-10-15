#include "ros/ros.h"
#include "std_msgs/String.h"
#include "can_msgs/CANFrame.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include "amu_3002a_lite/amu_control.h"

#define DEGtoRAD (3.14159265/180.0)
#define GtoMS2 9.80665F

double restoreAngle(unsigned char lsb, unsigned char msb);
double restoreAngleRate(unsigned char lsb, unsigned char msb);
double restoreAcc(unsigned char lsb, unsigned char msb);

ros::Publisher can_pub; // Publisher for /can_bus_tx
ros::Publisher data_pub;  // Publisher for /imu/data

double acc[3] = {0.0, 0.0, 0.0}; // [G]
double angle[3] = {0.0, 0.0, 0.0}; // [deg]
double dangle[3] = {0.0, 0.0, 0.0}; // [deg/sec]

/*
  Receive Data from CANUSB and restore
*/
void ReceiveDataCallback(const can_msgs::CANFrame::ConstPtr& msg)
{
  char buffer[10];
  
  sprintf(buffer, "%03X%d", msg->id, (int)msg->data.size());
  ROS_INFO("ID: %x (%d)", msg->id, (int)msg->data.size());
  
  std::string frame = buffer;
  for(int i = 0; i < (int)msg->data.size(); i++)
  {
    sprintf(buffer, "%02X", msg->data[i]);
    frame.append(buffer);
    ROS_DEBUG("D%d: %d", i, msg->data[i]);
  }
  
  // Angle
  if(msg->id == 0x41)
  {
    for(int i = 0; i < 3; i++)
    {
      angle[i] = restoreAngle(msg->data[i*2], msg->data[i*2+1]);
    }
    ROS_INFO("Roll: %3.1f Pitch: %3.1f Yaw: %3.1f[deg]", angle[0], angle[1], angle[2]);
  }
  else
  // Angule Rate
  if(msg->id == 0x42)
  {
    for(int i = 0; i < 3; i++)
    {
      dangle[i] = restoreAngleRate(msg->data[i*2], msg->data[i*2+1]);
    }
    ROS_INFO("Acc_R: %3.1f Acc_P: %3.1f Acc_Y: %3.1f[d/s]", dangle[0], dangle[1], dangle[2]);
  }
  else
  // Acceleration
  if(msg->id == 0x43)
  {
    for(int i = 0; i < 3; i++)
    {
      acc[i] = restoreAcc(msg->data[i*2], msg->data[i*2+1]);
    }
    ROS_INFO("Acc_x: %3.1f Acc_y: %3.1f Acc_z: %3.1f[m/s^2]", acc[0], acc[1], acc[2]);
  }
  
  // Publish Topic
  sensor_msgs::Imu data;
  // Quaternion
  double roll = angle[0] * DEGtoRAD;
  double pitch = -angle[1] * DEGtoRAD;
  double yaw = -angle[2] * DEGtoRAD;
  data.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  // Angule Rate[rad/s]
  data.angular_velocity.x = dangle[0] * DEGtoRAD;
  data.angular_velocity.y = dangle[1] * DEGtoRAD;
  data.angular_velocity.z = dangle[2] * DEGtoRAD;
  // Acceleration[m/s^2]
  data.linear_acceleration.x = acc[0] * GtoMS2;
  data.linear_acceleration.y = acc[1] * GtoMS2;
  data.linear_acceleration.z = acc[2] * GtoMS2;
  // Covariance
  data.orientation_covariance[0] = -1;
  data.angular_velocity_covariance[0] = -1;
  data.linear_acceleration_covariance[0] = -1;
  // Publish
  data_pub.publish(data);
}

/*
  Restore Angle Data [deg]
*/
double restoreAngle(unsigned char lsb, unsigned char msb)
{
  unsigned int datal = lsb;
  unsigned int datah = msb;
  double ret;
  int fdata = (datal + (datah << 8));
  if(fdata < 32768)
  {
    ret = fdata * 0.0078125;
  }
  else
  {
    ret = -(65536 - fdata) * 0.0078125;
  }
  return ret;
}

/*
  Restore Angle Rate Data [deg/sec]
*/
double restoreAngleRate(unsigned char lsb, unsigned char msb)
{
  unsigned int datal = lsb;
  unsigned int datah = msb;
  double ret;
  int fdata = (datal + (datah << 8));
  if(fdata < 32768)
  {
    ret = fdata * 0.015625;
  }
  else
  {
    ret = -(65536 - fdata) * 0.015625;
  }
  return ret;
}

/*
  Restore Acceleration Data [G]
*/
double restoreAcc(unsigned char lsb, unsigned char msb)
{
  unsigned int datal = lsb;
  unsigned int datah = msb;
  double ret;
  int fdata = (datal + (datah << 8));
  if(fdata < 32768)
  {
    ret = fdata * 0.0001220703125;
  }
  else
  {
    ret = -(65536 - fdata) * 0.0001220703125;
  }
  return ret;
}


/*
  Control AMU 3002A Lite
  command:
    0 : Stop
    1 : Start
    2 : Reset
  
  TODO:
    3 : SelfCheck
    4 : Software Version Information
*/
bool control(amu_3002a_lite::amu_control::Request &req, amu_3002a_lite::amu_control::Response &res)
{
  can_msgs::CANFrame msg;
  msg.id = 0x50;
  // Stop
  if(req.command == 0)
  {
    msg.data.resize(2);
    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    ROS_INFO("Send Stop command (%d)", req.command);
  }
  else
  // Start
  if(req.command == 1)
  {
    msg.data.resize(2);
    msg.data[0] = 0x01;
    msg.data[1] = 0x01;
    ROS_INFO("Send Start command (%d)", req.command);
  }
  // Reset
  if(req.command == 2)
  {
    msg.data.resize(2);
    msg.data[0] = 0x01;
    msg.data[1] = 0x01;
    ROS_INFO("Send Reset command (%d)", req.command);
  }
  // Publish
  can_pub.publish(msg);

  return true;
}


/*
  Main Function
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "amu_3002a_lite_node");

  ros::NodeHandle n;

  // Publisher
  can_pub = n.advertise<can_msgs::CANFrame>("/can_bus_tx", 1);
  data_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 1);
  // Subscriber
  ros::Subscriber sub = n.subscribe("/can_bus_rx", 3, ReceiveDataCallback);
  // Service
  ros::ServiceServer service = n.advertiseService("amu_control", control);
  
  // Wait until communication is establised
  ros::Duration(1.0).sleep();
  
  // Start measuring
  can_msgs::CANFrame msg;
  msg.id = 0x50;
  msg.data.resize(2);
  msg.data[0] = 0x01;
  msg.data[1] = 0x01;
  can_pub.publish(msg);
  ROS_INFO("Start measuring");
  
  ros::Rate loop_rate(100);
  // Main loop
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // Stop measuring
  msg.data[0] = 0x00;
  msg.data[1] = 0x00;
  can_pub.publish(msg);
  ROS_INFO("Stop measuring");
  
  return 0;
}
