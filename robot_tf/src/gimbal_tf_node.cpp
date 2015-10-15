#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "std_msgs/Float64.h"

// ロボット座標系で見たジンバルの回転中心の位置 [m]
#define GIMBAL_LOCATION_X 0.055
#define GIMBAL_LOCATION_Y 0.0
#define GIMBAL_LOCATION_Z 0.365//(0.365 - 0.050)

// ジンバルの回転中心から見たレーザー面の位置 [m]
#define GIMBAL2LASER_X 0.0
#define GIMBAL2LASER_Y 0.0
#define GIMBAL2LASER_Z 0.0264

double gimbal_angle; // ジンバル角度 [rad]
tf::Quaternion q;

void gimbalCallback(const std_msgs::Float64::ConstPtr& msg)
{                      
	gimbal_angle = msg->data;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_node");
	ros::NodeHandle n;
  
	ros::Subscriber sub_gimbang = n.subscribe("gimbal_angle", 1000, gimbalCallback);

	ros::Rate r(100);

	tf::TransformBroadcaster robot2gimbal_roll, robot2gimbal_pitch, gimbal2laser;
	tf::Quaternion q_urg, q_camera;
	ros::Time current_time;

	while(ros::ok()){
		
		current_time = ros::Time::now();

		robot2gimbal_roll.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0.5*sin(gimbal_angle), 0, 0), // 回転行列
				tf::Vector3(GIMBAL_LOCATION_X, GIMBAL_LOCATION_Y, GIMBAL_LOCATION_Z)),	// 移動
				current_time,	// タイムスタンプ
				"base_link",	// リンクの親ノードの名前
				"gimbal_link"	// 子ノードの名前
			)
		);
		robot2gimbal_pitch.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, 0.5*cos(gimbal_angle), 0), // 回転行列
				tf::Vector3(0, 0, 0)),	// 移動
				current_time,	// タイムスタンプ
				"gimbal_link",	// リンクの親ノードの名前
				"gimbal_link2"	// 子ノードの名前
			)
		);
		gimbal2laser.sendTransform(
			tf::StampedTransform( // 引数を５つとる
				tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), // 回転行列
				tf::Vector3(GIMBAL2LASER_X, GIMBAL2LASER_Y, GIMBAL2LASER_Z)),	// 移動
				current_time,	// タイムスタンプ
				"gimbal_link2",	// リンクの親ノードの名前
				"gimbal_laser"	// 子ノードの名前
			)
		);
	
	ros::spinOnce();
    r.sleep();
  }
}
