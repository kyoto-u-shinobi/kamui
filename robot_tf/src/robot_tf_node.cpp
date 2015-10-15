#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "calc_odometry/posdata.h"
#include "nav_msgs/Odometry.h"
#include "dynamixel_msgs/JointState.h"

// ロボット座標系で見たURGの位置 [m]
#define URG_LOCATION_X 0.171 //0.055
#define URG_LOCATION_Y 0.0
#define URG_LOCATION_Z 0.283 //0.372

// ロボット座標系で見た首振りDynamixelの位置 [m]
#define CAMERA_LOCATION_X -0.080 //-0.085
#define CAMERA_LOCATION_Y 0.0
#define CAMERA_LOCATION_Z 0.431 //0.445

// 首振りDynamixelから見た熱カメラの位置 [m]
#define CAMERA2THERMO_X 0.115
#define CAMERA2THERMO_Y 0.0
#define CAMERA2THERMO_Z 0.050

double x, y, z, roll, pitch, yaw; // オドメトリデータ
double camera_yaw;	// カメラ角度 [rad]
tf::Quaternion q;

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{                      
	camera_yaw = msg->current_pos;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	z = msg->pose.pose.position.z;

	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	//ROS_INFO("odomCallback");//debug
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_node");
	ros::NodeHandle n;
  
	ros::Subscriber sub_odom = n.subscribe("odom", 1000, odomCallback);
	ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);

	ros::Rate r(100);

	tf::TransformBroadcaster global2footprint, footprint2robot, robot2urg, robot2camera, camera2thermo;
	tf::Quaternion q_urg, q_camera;
	ros::Time current_time;

	while(ros::ok()){
		q_urg = tf::createQuaternionFromRPY(roll, pitch, 0);	//rollとpitchは水平に保たれている
		
		current_time = ros::Time::now();

		global2footprint.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, 0, yaw), // 回転行列
				tf::Vector3(x, y, 0)),	// 移動
				current_time,	// タイムスタンプ
				"odom",	// リンクの親ノードの名前
				"base_footprint"	// 子ノードの名前
			)
		);
		footprint2robot.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(q_urg, // 回転行列
				tf::Vector3(0, 0, z)),	// 移動
				current_time,	// タイムスタンプ
				"base_footprint",	// リンクの親ノードの名前
				"base_link"	// 子ノードの名前
			)
		);
		robot2urg.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::inverse(q_urg), // 回転行列（水平機構）
				tf::Vector3(URG_LOCATION_X, URG_LOCATION_Y, URG_LOCATION_Z)),	// 移動
				current_time,	// タイムスタンプ
				"base_link",	// リンクの親ノードの名前
				"laser_link"	// 子ノードの名前
			)
		);
		robot2camera.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, 0, camera_yaw), // 回転行列
				tf::Vector3(CAMERA_LOCATION_X, CAMERA_LOCATION_Y, CAMERA_LOCATION_Z)),	// 移動
				current_time,	// タイムスタンプ
				"base_link",	// リンクの親ノードの名前
				"camera_link"	// 子ノードの名前
			)
		);
		camera2thermo.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), // 回転行列
				tf::Vector3(CAMERA2THERMO_X, CAMERA2THERMO_Y, CAMERA2THERMO_Z)),	// 移動
				current_time,	// タイムスタンプ
				"camera_link",	// リンクの親ノードの名前
				"thermocam_link"	// 子ノードの名前
			)
		);
	
	ros::spinOnce();
    r.sleep();
  }
}
