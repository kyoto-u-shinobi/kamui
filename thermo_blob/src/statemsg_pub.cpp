//=================================================//
/*! @file
 @brief カメラユニットのテスト用状態msgのノード
 @author D.Furutani
 @date 2013/10/25
 @attention 

 @TODO 
*/
//=================================================//

using namespace std;

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/Int16.h"

// グローバル変数設定
int Scan_state = 0;	// カメラユニットの首振り状態
			// Scan_state=0:通常状態
			// Scan_state=1:強化探査状態
			// Scan_state=2:待機状態
			// Scan_state=3:通常状態でVictim発見
			// Scan_state=4:強化探査状態でVictim発見
			// Scan_state=5:Stop_temp状態
			// Scan_state=6:Stop_tempの折り返し条件を満たした状態
int state = 0;	// ロボットの状態
		// state=0:Normal
		// state=1:Tracking
		// state=2:Rotating
		// state=3:Closing
		// state=4:Scanning
		// state=5:GetInfo
		// state=6:Leaving
		// state=7:Stop_temp

int temp = 0;

//=============================================
//@name state_Callback
//@brief msgで受け取ったロボットの状態を変数に格納する
//@data 2013/10/25
//@attention
//=============================================	
void Scan_state_Callback(const std_msgs::Int16::ConstPtr& Scan_state_msg)
{

  // Scan_state=0:通常状態
  // Scan_state=1:強化探査状態
  // Scan_state=2:待機状態
  // Scan_state=3:通常状態でVictim発見
  // Scan_state=4:強化探査状態でVictim発見
  // Scan_state=5:Stop_temp状態
  // Scan_state=6:Stop_tempの折り返し条件を満たした状態

  // 取得を示すROS_INFOを流す
  ROS_INFO("Now get Scan_state_msg");

  // 取得したstate_msgを変数に格納する
  Scan_state = Scan_state_msg -> data;

  if (Scan_state == 3)
  {
	state = 1;
	ROS_INFO("Tracking Mode");
  }

  if (Scan_state == 2 && state == 1)	
  {
	state = 2;
	ROS_INFO("Rotating Mode");
  }
  
  if (Scan_state == 2 && state == 2)
  {
	state = 3;
	ROS_INFO("Closing Mode");
  }
  
  if (Scan_state == 2 && state == 3)
  {
	state = 4;
	ROS_INFO("Scanning Mode");
  }

  if (Scan_state == 4 && state == 4)
  {
	state = 5;
	ROS_INFO("Getinfo Mode");
  }

  if (state == 5)
  {
	state = 6;
	ROS_INFO("Leaving Mode");
  }

  if (state == 6)
  {
	state = 0;
	ROS_INFO("Normal Mode");
  }

  if (Scan_state == 0 && state == 0)
  {
	temp = temp + 1;
	if (temp > 10)
	{
		state = 7;
		temp = 1;
		ROS_INFO("Stop_temp Mode");
	}
  }
  else if (Scan_state == 6)
  {
	state = 0;
	ROS_INFO("Normal Mode");
  }
}

//=============================================
//@name main
//@brief メインループ
//@data 2013/10/25
//@attention
//=============================================	
int main(int argc, char **argv)
{
	// 各種設定

	// ROSの初期設定
	ros::init(argc, argv, "statemsg_pub");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);	// ループの待機時間(Hz)

	// 首振り状態送信用のパブリッシャーの定義
	ros::Publisher state_pub = n.advertise<std_msgs::Int16>("state", 100);
	std_msgs::Int16 state_msg;

	// State受信用のサブスクライバーの定義
	ros::Subscriber Scan_State_sub = n.subscribe("Scan_state", 100, Scan_state_Callback);

	// メインループ
	while (ros::ok())
	{
		// コールバックを呼び出して首振りの状態を取得
		ros::spinOnce();

		// ロボットの状態の配信
		state_msg.data = state;
		state_pub.publish(state_msg);

		// 次のデータを送る前にloop_rateだけ待機
		loop_rate.sleep();
	}
	return 0;
}
