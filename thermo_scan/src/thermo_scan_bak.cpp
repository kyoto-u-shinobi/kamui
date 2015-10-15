//=================================================//
/*! @file
 @brief カメラユニットの首振り処理のノード
 @author D.Furutani
 @date 2013/10/25
 @attention Dynamixcelを用いて首振りを行う

 @TODO プログラムの整理と待機時間の調整
*/
//=================================================//

using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include <std_msgs/Int32.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "dynamixel_msgs/JointState.h"
#include "set_state/pubState.h"
#include "dynamixel_controllers/SetSpeed.h"

#include <sstream>

#define SLEEP_TIME 0.5

// グローバル変数設定
bool blob_detect = false;	// 熱源発見の有無
bool Init_flag = true;		// ダイナミクセルの初期位置への移動フラグ

int Scan_state = 0;	// カメラユニットの首振り状態
			// Scan_state=0:通常状態
			// Scan_state=1:強化探査状態
			// Scan_state=2:待機状態
			// Scan_state=3:通常状態でVictim発見
			// Scan_state=4:強化探査状態でVictim発見
			// Scan_state=5:Stop_temp状態
			// Scan_state=6:Stop_tempの折り返し条件を満たした状態
std::string state = "Normal";	// ロボットの状態
		// state=0:Normal
		// state=1:Tracking
		// state=2:Rotating
		// state=3:Closing
		// state=4:Scanning
		// state=5:GetInfo
		// state=6:Leaving
		// state=7:Stop_temp
		// state=8:Waiting

//首ふり関連☆
//CAM_2は強化探査時のカメラユニットの動き
#define MIN_YAW_ANGLE_CAM	-(M_PI*100)/180	// センサユニットヨー角の最少角度(rad)
#define MAX_YAW_ANGLE_CAM	(M_PI*100)/180	// センサユニットヨー角の最大角度(rad)
#define ONE_STEP_ANGLE_CAM	(M_PI*20)/180	// １ステップ(rad)
#define ONE_STEP_ANGLE_CAM2	(M_PI*10)/180	// １ステップ(rad)

#define error_limit             (M_PI*3)/180    //  首振りDynamixel初期位置出しエラー判定値(rad)   
double error_cur = error_limit*2;

//#define WAGGING_SPEED_CAM	100	// １ステップの所要時間の1/4の値[ms]
//#define WAGGING_SPEED_CAM2	100	// １ステップの所要時間の1/4の値[ms]
//#define PROCESSING_TIME	600	// 画像処理の待機時間
//#define SCAN_SPEED_PI		200	// 熱画像のスキャン間隔[ms]

void DynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{                      
	error_cur = fabs(msg->error);
	ROS_INFO("current error: %f [rad]", error_cur);	//debug
}

//カメラユニットの首振り制御のクラス=========================================//
class ThermalScanning
{
private:
	int pm;			// pm=1:左回り、pm=-1:右回り
	double start_angle;	// スタート角度
	double end_angle;		// 折り返し角度
	double Dyna_angle;	// ダイナミクセルの角度
	int switch_num;	// stop_tempの時の折り返し回数


public:
	//bool cw;	// cw=true:右回り、cw=false:左回り
	int cw;

	ThermalScanning()
	{
		//cw = false;
	    cw=-1;
		switch_num = 0;

		Dyna_pub = n.advertise<std_msgs::Float64>("tilt_controller/command", 1000);
		//ros::Subscriber sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);
	}

	void InitScan(void);
	bool SuccessiveScan(void);

protected:
	ros::NodeHandle n;
	ros::Publisher Dyna_pub;
};

//=============================================================
//名前：InitScan
//説明：首振りの初期位置合わせ
//=============================================================
void ThermalScanning::InitScan(void)
{
	std_msgs::Float64 tilt_controller_command_msg; 

	if(cw == -1)	// 左回りの場合
	{
		start_angle = MIN_YAW_ANGLE_CAM;
		end_angle   = MAX_YAW_ANGLE_CAM;
		pm = 1;
	}else if(cw == 1){	// 右回りの場合
		start_angle = MAX_YAW_ANGLE_CAM;
		end_angle   = MIN_YAW_ANGLE_CAM;
		pm = -1;
	}
		
	// ダイナミクセルの角度を初期位置に移動する
	// TODO ダイナミクセルの角度をDyna_angleに変更する処理を入れる
	Dyna_angle = start_angle;

	tilt_controller_command_msg.data = Dyna_angle;
	
	error_cur = error_limit*2;
	while(error_cur > error_limit)
	{
		Dyna_pub.publish(tilt_controller_command_msg);
		//ros::Duration(SLEEP_TIME).sleep();
	}
}

//=============================================================
//名前：SuccessiveScan
//説明：熱源探知のための首振り
//=============================================================
bool ThermalScanning::SuccessiveScan(void)
{
	std_msgs::Float64 tilt_controller_command_msg; 	
	ros::ServiceClient client = n.serviceClient<dynamixel_controllers::SetSpeed>("tilt_controller/set_speed");

	// ダイナミクセルのスピードの設定
	dynamixel_controllers::SetSpeed srv;
	if(Scan_state == 0 || Scan_state == 5 || Scan_state == 6)	// 通常探索
	{ 
		srv.request.speed = 2.0;
	}
	else if(Scan_state == 1)	// 強化探索
	{
		srv.request.speed = 1.0;
	}	
	client.call(srv);

	// 首振りの処理
	if(pm*Dyna_angle < pm*end_angle - ((Scan_state == 1) ? ONE_STEP_ANGLE_CAM2 : ONE_STEP_ANGLE_CAM)) // 首振りがend_angle以下なら首振り
	{
		if(Scan_state == 1) // 強化探査状態
		{
			// TODO ダイナミクセルの角度をDyna_angleに変更する処理を入れる
			Dyna_angle = Dyna_angle  + (pm*ONE_STEP_ANGLE_CAM2);	// ONE_STEP_ANGLE_CAM2だけ首振り(強化探索)
			tilt_controller_command_msg.data = Dyna_angle;

			error_cur = error_limit*2;
			//while(error_cur > error_limit)
			{
				Dyna_pub.publish(tilt_controller_command_msg);
				ros::Duration(SLEEP_TIME).sleep();
			}
		}
		else
		{
			// TODO ダイナミクセルの角度をDyna_angleに変更する処理を入れる
			Dyna_angle = Dyna_angle + (pm*ONE_STEP_ANGLE_CAM);	// ONE_STEP_ANGLE_CAMだけ首振り
			tilt_controller_command_msg.data = Dyna_angle;

			error_cur = error_limit*2;
			//while(error_cur > error_limit)
			{
				Dyna_pub.publish(tilt_controller_command_msg);
				ros::Duration(SLEEP_TIME).sleep();
				//ROS_INFO("waiting... error: %f [rad]", error_cur);//debug
			}
		}
	}
	else if(pm*Dyna_angle >= pm*end_angle - ((Scan_state == 1) ? ONE_STEP_ANGLE_CAM2 : ONE_STEP_ANGLE_CAM))
	{
		ROS_INFO("4");

		//cw = !cw;
		cw = -cw;
		InitScan();	// 首振り方向を反転させて初期位置へ移動

		ROS_INFO("5");

		// Stop_temp状態の場合の首振り回数カウント
		if (Scan_state == 5)
		{
			switch_num = switch_num + 1;
			if (switch_num >= 2)
			{
				switch_num = 0;
				return true;	// Stop_temp終了ならtrueを返す
			}
		}
		
	return false;
	}
}

//=============================================
//@name Blob_detect_Callback
//@brief msgで受け取った熱源の有無を変数に格納する
//@data 2013/10/25
//@attention
//=============================================	
void Blobcallback(const std_msgs::Bool::ConstPtr& blobmsg)
{ 
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<set_state::pubState>("pubState");

	set_state::pubState srv;
  
	// 通常、強化探査、Stop_temp状態でVictimを検出したら検出状態へ移行
	if (blobmsg->data)
	{  
		if (Scan_state == 0 ||Scan_state == 5 || Scan_state == 6)
		{
			Scan_state = 3;
			ROS_INFO("Victim Found!(Normal)");
        	state = "Tracking";
			srv.request.state = state;
			client.call(srv);
		}
		else if (Scan_state == 1)
		{
			Scan_state = 4;
			ROS_INFO("Victim Found!(Scanning)");
			state = "Waiting";
			srv.request.state = state;
			client.call(srv);
			//TODO: GUIにVictim発見情報を送る
		}
	}  
}

//=============================================
//@name state_Callback
//@brief msgで受け取ったロボットの状態を変数に格納する
//@data 2013/10/25
//@attention
//=============================================	
void state_Callback(const std_msgs::String::ConstPtr& state_msg)
{
	// 取得したstate_msgを変数に格納する
	state = state_msg -> data;

	// 強化探査、待機、Stop_temp(終了)状態から通常状態への移行
	if ((Scan_state == 1 || Scan_state == 2 || Scan_state == 4 || Scan_state == 6) && state == "Normal")
	{
		Scan_state = 0;
		Init_flag = true;
	}
	// 待機もしくはVictim検出(通常)状態から強化探査状態への移行
	else if ((Scan_state == 2 || Scan_state == 3) && state == "Scanning")	
	{
		Scan_state = 1;
		Init_flag = true;
	}
	// 待機状態への移行
	else if (state == "Tracking" || state == "Rotating" || state == "Closing" || state == "GetInfo" || state == "Leaving")
	{
		Scan_state = 2;
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
	// ROSの初期設定
	ros::init(argc, argv, "thermo_scan");

	// 各種設定
	ThermalScanning ts;	// 首振りクラスの定義

	ROS_INFO("1");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);	// ループの待機時間(Hz)

	// 首振り状態送信用のパブリッシャーの定義
	ros::Publisher Scan_state_pub = n.advertise<std_msgs::Int16>("Scan_state", 100);
	ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 1000, DynaCallback);
	std_msgs::Int16 Scan_state_msg;

	// Victim検出結果受信用のサブスクライバーの定義
	ros::Subscriber Blob_detect_sub = n.subscribe("optris/ThermoBlob", 100, Blobcallback);

	// State受信用のサブスクライバーの定義
	ros::Subscriber State_sub = n.subscribe("state", 100, state_Callback);

	ROS_INFO("2");

	int count=0;

	// ダイナミクセルの初期化
	if (Init_flag == true)
	{
		Init_flag == false;
		ts.cw = -1;
		ts.InitScan();
	}

	// メインループ
	while (ros::ok())
	{
		// コールバックを呼び出してVictim検出結果とロボットの状態を取得
		ros::spinOnce();

		// スキャン状態が通常、強化探査、Stop_tempなら首振り
		if (Scan_state == 0 || Scan_state == 1 || Scan_state == 5 || Scan_state == 6)
		{
			if(ts.SuccessiveScan() == true)
			{
				Scan_state == 6;
				count++;
			}
		}

		ROS_INFO("Scan_state: %d", Scan_state);//debug

		// カメラユニットの首振り状態の配信
		Scan_state_msg.data = Scan_state;
		Scan_state_pub.publish(Scan_state_msg);                

		// 次のデータを送る前にloop_rateだけ待機
		loop_rate.sleep();
	}
	return 0;
}
