#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include <string>
#include "wall_follower/walls.h"

//各種定義
#define Rw  0.095					/*! 車輪半径[m] (直径0.19m)*/
#define T  0.230					/*!トレッド[m]             */
#define CRAWLERFULL  6.0//12.0			/*! クローラの構造上の最大回転速度[rad/s]*/
#define CRAWLERLIMIT 1.5//3.0			/*! クローラの制限最大回転速度[rad/s]    */

#define W_ADJUST 1.00				/*! 回転に関する係数（オドメトリ）  */

#define	ROAD_WIDTH	800//650//850		//!< 道の幅[m]//900mm...(JapanOpen),1200mm(WorldCup)

//☆ロボットの筐体，自律走行のパラメータ
#define	ROBOT_WIDTH 350				/*横幅*/
#define	ROBOT_DEPTH 498				/*奥行き*/  
#define	MinF 450//480//370//370 //270 //ROAD_WIDTH/2-100,				/*前方の閾値 MinL以上MaxL以下 */        
#define	MaxL 520//520//400//350//ROAD_WIDTH/2+100 //450				/*左方の閾値(max)*/   
#define	MinL 300//450//340//325  //ROAD_WIDTH/3, //-50,//300						//600 左方の閾値(min)
//#define	SMaxL ROAD_WIDTH/2 + 50  //400 		/*左方の閾値(smax)*/  
//#define	TRANSLATIONAL_VEL 32//28//14 //10,//7,//10,//25//32			//並進速度(kamuiへの指令値になっている+-100の範囲)    弄ります by rocket
int TRANSLATIONAL_VEL = 40;
//#define	ROTATIONAL_VEL 32//28//14//10//7,//15				//角速度(kamuiへの指令値になっている+-100の範囲)
	//22//ACアダプター使用時はこの20以上に設定すると電流が不安定になる
int ROTATIONAL_VEL = 36;
#define RATE_TRANS	0	

#define y_RANGE	170 // 前方壁認識の横方向範囲
  
int Count;		//!<はまりカウンタ
char L_R;		//!<はまりを回避するためのもの	

//====================================================//
/*!
*	@class 距離データの定義
*/
//====================================================//
class Distance{
public:
	Distance(){
		Clear();
	}
	~Distance(){
	}
	void Clear(void){//適当に大きな値で初期化
		front  = 10000;
		left   = 10000;
		right  = 10000;
		left2  = 10000;
		right2 = 10000;
	}

public:
	double front;	//前方 [mm]
	double left;	//左前 [mm]
	double right;	//右前 [mm]
	double left2;	//左後 [mm]
	double right2;	//右後 [mm]
};

Distance dist;

//-----------------------------------------------------
// URGデータのコールバック関数
// 計測距離データをロボット座標での3次元点データに変換したものから
// ロボット周囲の障害物までの距離を計算する
//-----------------------------------------------------
void wallsCallback(const wall_follower::walls::ConstPtr& msg)
{
	dist.Clear();
	
	dist.front = msg->front;
	dist.left = msg->left;
	dist.left2 = msg->left2;
	dist.right = msg->right;
	dist.right2 = msg->right2;
	
	//ROS_INFO("%f %f %f %f %f", dist.front, dist.right, dist.left, dist.right2, dist.left2);
}

//=========================================================================//
//名前：CalcAutoVelocityLeft2
//説明：senseAroundで計算した値から制御入力を計算(左手法はやめ)
/*!
*	@brief senseAroundで計算した値から制御入力を計算(左手法ゆっくりめ)
*	@param
*	@retval void
*/
//=========================================================================//
void CalcAutoVelocityLeft2(int *vel0, int *avel0)
{
	if(dist.right>ROAD_WIDTH/2+200){//右壁が遠いと仮想の壁を作成
		dist.right=ROAD_WIDTH/2+200;
	}else if(dist.right2>ROAD_WIDTH/2+200){
		dist.right2=ROAD_WIDTH/2+200;
	}
    double V_input=0;			//並進速度
	double W_input=0;			//回転速度
//	double Vr = -5;				//旋回中のときの並進速度
	
	//＜第一優先：前方＞前方に壁があれば．．．
	if (dist.front < MinF)
	{
//		printf("前方に壁のフラグ\n");
		if(dist.left > MaxL/**1.414*1.2*/){					//左に壁がなければ左旋回(この値は旋回途中で最も離れる距離＋α)
			if(dist.right < MaxL){
				W_input = ROTATIONAL_VEL;
				V_input = 0;
			}
			else{
				W_input = -ROTATIONAL_VEL;
				V_input = 0;
			}
		}else if(dist.right > MaxL){			//左に壁あり、右に壁がなければ右旋回
			W_input = -ROTATIONAL_VEL;
			V_input = 0;
		}else{									//前も、左右も壁→左手法なので右旋回
			V_input = 0;
			W_input = -ROTATIONAL_VEL;
		}
	}
	//＜第二優先：左壁＞
	else if(dist.left <= MinL || dist.left2<=MinL)			//左壁にとても近い
	{	
//		printf("左壁を沿います\n");
		if(dist.left - dist.left2<-50){	//左壁方向に向いていたら離れるように
			V_input = RATE_TRANS*TRANSLATIONAL_VEL;
		    W_input = -ROTATIONAL_VEL;
		}
		else{						//左壁から離れる方向であれば直進
			V_input = TRANSLATIONAL_VEL;
			W_input = 0;
		}
	}
	else if((dist.left >MinL || dist.left2>MinL)&&(dist.left <= MaxL || dist.left2<=MaxL) )		//左壁にまぁまぁ近い
	{	
//		printf("左と離れすぎなので近寄ります\n");
		if(dist.left > dist.left2){	//左壁方向にから離れる方向に向いてるのであれば旋回
			V_input = RATE_TRANS*TRANSLATIONAL_VEL;
		    W_input = ROTATIONAL_VEL;
		}
		else{						//左壁に近づく方向であれば直進
			V_input = TRANSLATIONAL_VEL;
			W_input = 0;
		}
	}
	else												//左の壁から遠い場合
	{
		W_input = ROTATIONAL_VEL;
		V_input = 0;//TRANSLATIONAL_VEL;
	}

	//戻り値にデータコピー
	*vel0 = (int)V_input;
	*avel0 = (int)W_input;
}
//=========================================================================//
//名前：CalcAutoVelocityRight2
//説明：senseAroundで計算した値から制御入力を計算(右手法はやめ)
/*!
*	@brief senseAroundで計算した値から制御入力を計算(右手法はやめ)
*	@param
*	@retval void
*/
//=========================================================================//
void CalcAutoVelocityRight2(int *vel0, int *avel0)
{
/*	if(dist.left>ROAD_WIDTH/2+200){//左壁が遠いと仮想の壁を作成
		dist.left=ROAD_WIDTH/2+200;
	}else if(dist.left2>ROAD_WIDTH/2+200){
		dist.left2=ROAD_WIDTH/2+200;
	}*/
    double V_input=0;			//並進速度
	double W_input=0;			//回転速度
	
	////＜第一優先：前方＞前方に壁があれば．．．
	if (dist.front < MinF)
	{
		if(dist.right > MaxL/**1.414*1.2*/){					//右に壁がなければ右旋回(この値は旋回途中で最も離れる距離＋α)
			if(dist.left < MaxL){
				W_input = -ROTATIONAL_VEL;
				V_input = 0;
				}
			else{
				W_input = ROTATIONAL_VEL;
				V_input = 0;
			}
		}else if(dist.left > MaxL){			//右に壁あり、左に壁がなければ左旋回
			W_input = ROTATIONAL_VEL;
			V_input = 0;
		}else{									//前も、左右も壁→右手法なの左旋回
			V_input = 0;
			W_input = ROTATIONAL_VEL;
		}
	}
	//＜第二優先：左壁＞
	else if(dist.right <= MinL || dist.right2<=MinL)			//左壁にとても近い
	{	
		if(dist.right - dist.right2<-50){	//左壁方向に向いていたら離れるように
			V_input = RATE_TRANS*TRANSLATIONAL_VEL;
		    W_input = ROTATIONAL_VEL;
		}
		else{						//左壁から離れる方向であれば直進
			V_input = TRANSLATIONAL_VEL;
			W_input = 0;
		}
	}
	else if((dist.right >MinL || dist.right2>MinL)&&(dist.right <= MaxL || dist.right2<=MaxL) )		//左壁にまぁまぁ近い
	{	
		if(dist.right > dist.right2){	//左壁方向にから離れる方向に向いてるのであれば旋回
			V_input = RATE_TRANS*TRANSLATIONAL_VEL;
		    W_input = -ROTATIONAL_VEL;
		}
		else{						//左壁に近づく方向であれば直進
			V_input = TRANSLATIONAL_VEL;
			W_input = 0;
		}
	}
	else												//左の壁から遠い場合
	{
		W_input = -ROTATIONAL_VEL;
		V_input = 0;//TRANSLATIONAL_VEL;
	}

	//戻り値にデータコピー
	*vel0 = (int)V_input;
	*avel0 = (int)W_input;
}

//=========================================================================//
//名前：SelecVelocity
//説明：環境に応じて使用する関数を選択・速度指令値を出力する(後藤君作のを移植)
/*!
*	@brief　環境に応じて使用する関数を選択・速度指令値を出力する(後藤君作のを移植)
*	@param  vel, avel	// 速度指令コマンド
*	@retval void 
*	@attenrion 右左大丈夫(あってる？？)??
*/
//=========================================================================//
void SelectRun(int *vel, int *avel)
{
	int vel0, avel0;
	int flag;
	std::string mode;	
	
	ros::param::get("wall_follower_node/right_or_left", mode);
	if(mode == "left")	//左手
	{
		ROS_INFO("left");//debug
		CalcAutoVelocityLeft2(&vel0, &avel0);
		flag = 1;
	}
	else if(mode == "right")	//右手
	{
		CalcAutoVelocityRight2(&vel0, &avel0);
		flag = 2;
	}
	else
	{
		vel0 = 0;
		avel0 = 0;
		flag = 0;
	}
		
	if(flag != 0)
	{
		//はまり解除モード
		char preL_R = L_R;
		if(avel0 > 0) L_R = 'L';
		else if(avel0 < 0) L_R = 'R';
		else L_R = 'N';
		if(vel0 >= 0.7*TRANSLATIONAL_VEL || L_R=='N') Count = 0;
		else if(preL_R != L_R) Count++;	
		if(Count >=6 && Count <=20)
		{
			Count++;
			vel0 = 0;
			if(flag == 1)
				avel0 = -ROTATIONAL_VEL;//左手
			else
				avel0 = ROTATIONAL_VEL;//右手
			printf("はまり回避モード\n");
		}else if(Count > 20) Count=0;
	}

	*vel = vel0;
	*avel = avel0;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wall_follower_node");

	ros::NodeHandle n;

	ros::Subscriber walls_sub = n.subscribe("walls", 1000, wallsCallback);
	ros::Publisher pub_cmdvel = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);

	geometry_msgs::Twist cmd;
	int vel, avel;	// 速度指令コマンド

	if(ros::param::has("/wall_follower_node/vel"))
	{
		ros::param::get("/wall_follower_node/vel", TRANSLATIONAL_VEL);
	}
	if(ros::param::has("/wall_follower_node/avel"))
	{
		ros::param::get("/wall_follower_node/avel", ROTATIONAL_VEL);
	}

	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		SelectRun(&vel, &avel);	//速度入力を計算

		cmd.linear.x = (double)vel * (Rw * CRAWLERFULL) / 100;
		cmd.angular.z = (double)avel * (Rw * CRAWLERFULL * 2 / (T * W_ADJUST)) / 100;
		
		pub_cmdvel.publish(cmd);
		ROS_INFO("vel: %d   avel:%d", vel, avel);//debug

		ros::spinOnce();

		loop_rate.sleep();
	}
}
