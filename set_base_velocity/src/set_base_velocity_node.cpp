#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "kamui_control/base_velocity.h"
#include "dynamixel_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include "set_state/pubState.h"
#include "std_msgs/String.h"
#include "calc_odometry/posdata.h"

//各種定義
#define Rw  0.095					/*! 車輪半径[m] (直径0.19m)*/
#define T  0.230					/*!トレッド[m]             */
#define CRAWLERFULL  6.0//12.0			/*! クローラの構造上の最大回転速度[rad/s]*/
#define CRAWLERLIMIT 1.5//3.0			/*! クローラの制限最大回転速度[rad/s]    */
#define	ROBOT_WIDTH	0.35			/*ロボットの横幅*/

#define W_ADJUST 1.00				/*! 回転に関する係数（オドメトリ）  */

#define TRANSLATIONAL_VEL 36//10		/*並進速度*/
#define ROTATIONAL_VEL 36//10           /*回転速度*/
#define DISTCONST		0.5			/*離脱モードの閾値*/
#define ANGCONST		75.0*M_PI/180.0			/*離脱モードのangle閾値*/
#define CLOSE_TO_VICTIM 0.38		//380mm以内になるまで近づく [m]

#define CONTROL_F 50//10	// 制御周期 [Hz]

#define ROTATE_ANGLE 60.0*M_PI/180.0

double linearVel, angularVel;	// 速度指令値（並進、回転）
int linearVel_tele, angularVel_tele;	// 速度指令値（並進、回転）teleoperating
double cur_dyna_pos;	// 首振りDynamixelの角度
double cur_yaw;			// ロボットのyaw角度
double cur_x, cur_y;	// ロボットの現在位置
double front_dist;		// ロボット前方の障害物までの距離

std::string state = "Waiting";	// ロボットの状態

//=============================================
//@name state_Callback
//@brief msgで受け取ったロボットの状態を変数に格納する
//@data 2013/10/25
//@attention
//=============================================	
void state_Callback(const std_msgs::String::ConstPtr& state_msg)
{
	state = state_msg -> data;
}

//-----------------------------------------------------
// URGデータのコールバック関数
// 計測距離データをロボット座標での3次元点データに変換したものから
// ロボット前方の障害物までの距離を計算する
//-----------------------------------------------------
void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	double dist_min = 10000;	//適当な大きい数で初期化
	for(int i = 0; i < (int)msg->points.size(); i++){ 
		if(msg->points[i].x >= 0.1){ //ロボットの中心より10cm以上前方
			if( (msg->points[i].y > -(ROBOT_WIDTH/2+0.1)) && (msg->points[i].y < +(ROBOT_WIDTH/2+0.1)) )
			{
				if(msg->points[i].x < dist_min)
				{
					dist_min = msg->points[i].x;					
				}
			}
		}
	}
	front_dist = dist_min;
}


//-----------------------------------------------------
// オドメトリデータのコールバック関数
//-----------------------------------------------------
/*
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	cur_x = msg->pose.pose.position.x;
	cur_y = msg->pose.pose.position.y;
	cur_yaw = tf::getYaw(msg->pose.pose.orientation);
	ROS_INFO("cur_yaw: %f", cur_yaw);//debug
}
*/
void roboposCallback(const calc_odometry::posdata::ConstPtr& msg)
{
	cur_x = msg->x;
	cur_y = msg->y;
	cur_yaw = msg->yaw;
	//ROS_INFO("cur_yaw: %f", cur_yaw);//debug
}

//-----------------------------------------------------
// 首振りDynamixelのコールバック関数
//-----------------------------------------------------
void dynaCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
	cur_dyna_pos = msg->current_pos;
}

//-----------------------------------------------------
// 速度指令コマンドのコールバック関数
//-----------------------------------------------------
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	linearVel = msg->linear.x;
	angularVel = msg->angular.z;
	//ROS_INFO("cmdVelCallback");	//debug
}

//-----------------------------------------------------
// 速度指令コマンドのコールバック関数(Teleoperating)
//-----------------------------------------------------
void televel_Callback(const kamui_control::base_velocity::ConstPtr& msg)
{
	linearVel_tele = msg->linear;
	angularVel_tele = msg->angular;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "set_base_velocity_node");

	ros::NodeHandle n;
	kamui_control::base_velocity bvel;
	double pre_x, pre_y, pre_yaw;		// Leaving直前のロボットの位置、姿勢
	double dyna_target;		// Rotatingの目標角度

	ros::Subscriber urg_sub = n.subscribe("cloud", 50, cloudCallback);
	//ros::Subscriber odom_sub = n.subscribe("odom", 50, odomCallback);
	ros::Subscriber robopos_sub = n.subscribe("robot_pos", 50, roboposCallback);
	ros::Subscriber dyna_sub = n.subscribe("tilt_controller/state", 50, dynaCallback);
	ros::Subscriber cmdvel_sub = n.subscribe("cmd_vel", 50, cmdVelCallback);
	ros::Subscriber state_sub = n.subscribe("state", 1000, state_Callback);
	ros::Subscriber televel_sub = n.subscribe("Velocity_tele", 50, televel_Callback);
	ros::Publisher pub = n.advertise<kamui_control::base_velocity>("Velocity", 50);

	ros::ServiceClient client = n.serviceClient<set_state::pubState>("pubState");

	set_state::pubState srv;


	ros::Rate loop_rate(CONTROL_F);

	bvel.linear = 0;
	bvel.angular = 0;
	
	//ros::param::set("state", 0); // とりあえずNormal

	while (ros::ok())
	{
		if(state == "Normal")	// Normal
		{
			bvel.linear = (int)(linearVel / (Rw * CRAWLERFULL) * 100);
			bvel.angular = (int)(angularVel / (Rw * CRAWLERFULL * 2 / (T * W_ADJUST)) * 100);
		}
		else if(state == "Tracking")	// Tracking
		{
			bvel.linear = 0;
			bvel.angular = 0;
			pre_yaw = cur_yaw;
			dyna_target = cur_dyna_pos;
			ROS_INFO("Tracking...cur_yaw: %f  pre_yaw:%f  dyna_target:%f", cur_yaw, pre_yaw, dyna_target);//debug
			srv.request.state = "Rotating";
			client.call(srv); // ROtatingモードへ
		}
		else if(state == "Rotating")	// Rotating
		{
			//ROS_INFO("Rotating...  cur_yaw:%f, pre_yaw:%f", cur_yaw, pre_yaw);//debug
			if(fabs(cur_yaw - pre_yaw) < fabs(dyna_target))	// 発見してから回転した角度が，ターゲット発見時のカメラの角度を超えると止まる
			//if(fabs(cur_yaw - pre_yaw) < ROTATE_ANGLE)	// 発見してから回転した角度が，ターゲット発見時のカメラの角度を超えると止まる
			{
				//回転方向を変える
				bvel.linear = 0;
				bvel.angular = (dyna_target > 0)?ROTATIONAL_VEL:-ROTATIONAL_VEL;
			}
			else
			{//ある程度回転したら
				ROS_INFO("Rotating...cur_yaw: %f  pre_yaw:%f  dyna_target:%f", cur_yaw, pre_yaw, dyna_target);//debug
				srv.request.state = "Closing";
				client.call(srv); // Closingモードへ
			}
		}
		else if(state == "Closing")		// Closing
		{
			if(front_dist > CLOSE_TO_VICTIM)
			{
				bvel.linear = 2*TRANSLATIONAL_VEL;
				bvel.angular = 0;
				//ROS_INFO("Closing");//debug
			}
			else
			{
				bvel.linear = 0;	
				bvel.angular = 0;
				srv.request.state = "Scanning";
				client.call(srv); // Scanningモードへ
			}
		}
		else if(state == "Scanning")	// Scanning
		{
			bvel.linear = 0;
			bvel.angular = 0;
			//ROS_INFO("Scanning");//debug
		}
		else if(state == "GetInfo")	// GetInfo
		{
			bvel.linear = 0;
			bvel.angular = 0;
			pre_x = cur_x;
			pre_y = cur_y;
			pre_yaw = cur_yaw;
		}
		else if(state == "Leaving")	// Leaving
		{
			bvel.linear = (int)(linearVel / (Rw * CRAWLERFULL) * 100);
			bvel.angular = (int)(angularVel / (Rw * CRAWLERFULL * 2 / (T * W_ADJUST)) * 100);
			ROS_INFO("Leaving");//debug
			if(sqrt(pow((cur_x-pre_x),2.0)+pow((cur_y-pre_y),2.0)) > DISTCONST)// || fabs(cur_yaw - pre_yaw) > ANGCONST + fabs(dyna_target))	// 熱源位置から一定距離離れるまでLeavingモード
			{
				//cout<<mess.Get_vicnum()+1<<"体目を見つけに行きます"<<endl;
				/*
				if(ros::param::has("/frontier"))
				{
					srv.request.state = "Following";
				}
				else
				{
					srv.request.state = "Normal";
				}
				*/
				
				srv.request.state = "Following";
				client.call(srv); // Followingモードへ
			}
		}
		else if(state == "Stop_temp")	// Stop_temp
		{	
			bvel.linear = 0;
			bvel.angular = 0;
		}
		else if(state == "Waiting")	// Waiting
		{
			bvel.linear = 0;
			bvel.angular = 0;
		}
		else if(state == "Following")	// Following
		{
			bvel.linear = (int)(linearVel / (Rw * CRAWLERFULL) * 100);
			bvel.angular = (int)(angularVel / (Rw * CRAWLERFULL * 2 / (T * W_ADJUST)) * 100);
		}
		else if(state == "Moving")	// Moving
		{
			bvel.linear = (int)(linearVel / (Rw * CRAWLERFULL) * 100);
			bvel.angular = (int)(angularVel / (Rw * CRAWLERFULL * 2 / (T * W_ADJUST)) * 100);
		}
		else if(state == "Teleope")	// Teleoperating
		{
			bvel.linear = linearVel_tele;
			bvel.angular = angularVel_tele;
		}
		else
		{
			bvel.linear = 0;
			bvel.angular = 0;
		}
	
		pub.publish(bvel);
		//ROS_INFO("bvel linear: %d,  angular: %d", (int)bvel.linear, (int)bvel.angular);//debug

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
