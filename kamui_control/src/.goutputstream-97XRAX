#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"
#include "sh2interface/sh2_tx.h"
#include "kamui_control/flipper_velocity.h"
#include "kamui_control/base_velocity.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

//ロボットに関する定数
#define Rw  0.095					/*! 車輪半径[m] (直径0.19m)*/
#define T  0.230					/*!トレッド[m]             */
#define CRAWLERFULL  12.0			/*! クローラの構造上の最大回転速度[rad/s]*/
#define CRAWLERLIMIT 3.0			/*! クローラの制限最大回転速度[rad/s]    */
#define EMPTY  0x0000				/*! empty(マイコンとの通信で使用)        */

//オドメトリに関する定数
#define SETTING_ANGLE_PITCH 1.1		/*! ICのpitch取り付け角[deg]      */
#define THRESHOLD_PITCH 0.0			/*! 坂と判定するpitch閾値[deg]    */

#define SETTING_ANGLE_ROLL 0.0		/*! ICのroll取り付け角[deg]      */
#define THRESHOLD_ROLL 0.0			/*! roll角度反映の閾値[deg]　　  */

#define VEL_THRESHOLD 5			/*! 並進速度に関する不感帯 */
#define AVEL_THRESHOLD 5//19			/*! 角速度に関する不感帯   */

#define ADJUST_F 3.20//1.55				/*! 前進に関するエンコーダの係数 */
#define ADJUST_B 1.60//0.85//0.901			/*! 後進に関するエンコーダの係数   */
#define W_ADJUST 3.00				/*! 回転に関する係数             */

#define THRESH_ODD_SENSOR 0.001		/*!	オドメトリとIC3を切り替える閾値[rad]*/

#define PLUSE_PER_ROUND_C 136970.24       /*! クローラのエンコーダのパルス数[pulse/round]  */
#define PLUSE_PER_ROUND_F 136970.24*5.7	  /*! フリッパーのエンコーダのパルス数[pulse/round]*/

#define SIZE_DRIFT 20

sh2interface::sh2_tx txdata;

//=======================================================================================================//
//速度関連のクラス
//=======================================================================================================//
class VelData
{
public:
	//変数
	int vel;		//最大値を100とする(並進)
	int avel;		//最大値を100とする(回転)
	int fvelleft;	//最大値を100とする（フリッパー右）
	int fvelright;	//最大値を100とする（フリッパー左）
	int times;		//倍速の設定

	VelData(void)
	{
		Clear();
	}
	~VelData(void){	};

	void Clear(void)
	{
		vel  = 0;
		avel = 0;
		fvelleft = 0;
		fvelright = 0;
		times = 1;
	}
	//オペレータのオーバーロード
	VelData operator=(VelData &v1){
		vel = v1.vel;
		avel = v1.avel;
		fvelleft = v1.fvelleft;
		fvelright = v1.fvelright;
		times = v1.times;
		return *this;
	}
};

//==================================================================//
//姿勢データクラス
//=========------===================================================//
class RotData
{
public:
	ros::NodeHandle n;

	double roll;	//ロール角[rad]
	double pitch;	//ピッチ角[rad]
	double yaw;		//ヨー角  [rad]

	RotData(){
		ros::NodeHandle n;
		pose_sub = n.subscribe("imu/data", 1000, &RotData::poseCallback, this);
		Clear();
	};
	~RotData(){};
	
	virtual void Clear()
	{
		roll  = 0.0;
		pitch = 0.0;
		yaw   = 0.0;
	}	
	RotData operator=(RotData &rot0){
		roll  = rot0.roll;
		pitch = rot0.pitch;
		yaw   = rot0.yaw;
		return *this;
	}

	void poseCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// amuから姿勢データ受信
		tf::Quaternion q;
	
		tf::quaternionMsgToTF(msg->orientation, q); // Quaternion msgからtf::Quaternionに変換
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);	// tf::Quaternionからroll, pitch, yawを取得
		ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
	}
	
protected:
	ros::Subscriber pose_sub;
};

//=======================================================================================================//
//KamuiControlクラス
//=======================================================================================================//
class KamuiControl : virtual public RotData
{
public:
	RotData rot;

	KamuiControl(){
		ros::NodeHandle n;
		pub = n.advertise<sh2interface::sh2_tx>("sh2_tx", 1000);
		vel_sub = n.subscribe("Velocity", 10, &KamuiControl::VelocityCallback, this);
		fvel_sub = n.subscribe("Flipper_Velocity", 10, &KamuiControl::fVelocityCallback, this);
	}
	
	~KamuiControl(){};


	void VelocityCallback(const kamui_control::base_velocity::ConstPtr& msg)
	{
		// とりあえず現在のKAMUIプログラムに合わせる形で
		VelData veld, veld_tx;
		//veld.vel = (int)(msg->linear.x / (Rw * CRAWLERLIMIT) * 100);
		//veld.avel = (int)(msg->angular.z / (Rw * CRAWLERLIMIT * 2 / (T *  W_ADJUST)) * 100);
		veld.vel = msg->linear;
		veld.avel = msg->angular;
	
		// limitter	
		if(veld.vel > 100)	veld.vel = 100;
		else if(veld.vel < -100) veld.vel = -100;
		if(veld.avel > 100)	veld.avel = 100;
		else if(veld.avel < -100) veld.avel = -100;

		sh2interface::sh2_tx txdata;
		int16_t data1, data2;

		//veld = SlopeCorrectionVel(veld);
		SlopeCorrectionVel(veld, &veld_tx);

		//ROS_INFO("%d %d", veld_tx.vel, veld_tx.avel);//debug -> OK
		ControlFromVelData(veld, &data1, &data2);

		//ROS_INFO("%d %d", data1, data2);//debug -> OK

		txdata.command = 0x13;
		txdata.id = 0x01;
		txdata.data[0] = (uint8_t)(data2 >> 8);
		txdata.data[1] = (uint8_t)data2;
		txdata.data[2] = (uint8_t)(data1 >> 8);
		txdata.data[3] = (uint8_t)data1;

		ROS_INFO("crawler:%x %x %x %x", txdata.data[0], txdata.data[1], txdata.data[2], txdata.data[3]);//debug
		pub.publish(txdata);
	}

	void fVelocityCallback(const kamui_control::flipper_velocity::ConstPtr& msg)
	{
		sh2interface::sh2_tx txdata;
		int16_t data1, data2;
		double fvelR, fvelL;

		fvelR = msg->fvelright;
		fvelL = msg->fvelleft;

		// limitter	
		if(fvelR > CRAWLERLIMIT)	fvelR = CRAWLERLIMIT;
		else if(fvelR < -CRAWLERLIMIT)	fvelR = -CRAWLERLIMIT;
		if(fvelL > CRAWLERLIMIT)	fvelL = CRAWLERLIMIT;
		else if(fvelL < -CRAWLERLIMIT)	fvelL = -CRAWLERLIMIT;

		data1 = (int16_t)(fvelR / CRAWLERFULL * 32767);
		data2 = -(int16_t)(fvelL / CRAWLERFULL * 32767);	// ハードウェアの都合で符号逆転

		txdata.command = 0x13;	
		txdata.id = 0x11;
		txdata.data[0] = (uint8_t)(data2 >> 8);
		txdata.data[1] = (uint8_t)data2;
		txdata.data[2] = (uint8_t)(data1 >> 8);
		txdata.data[3] = (uint8_t)data1;

		ROS_INFO("flipper:%x %x %x %x", txdata.data[0], txdata.data[1], txdata.data[2], txdata.data[3]);//debug
		pub.publish(txdata);
	}
	
	void SlopeCorrectionVel(VelData vel0, VelData* veld);
	void ControlFromVelData(VelData veld0, int16_t* data1, int16_t* data2);

protected:
	ros::Publisher pub;
	ros::Subscriber vel_sub;
	ros::Subscriber fvel_sub;
};

//=======================================================================================================//
//名前：SlopeCorrectionVel
//説明：3次元の姿勢からピッチ坂、ロール坂での旋回時の滑りを考慮する関数
/*!
	@brief 
	@param 
	@retval 
*/
//=======================================================================================================//
void KamuiControl::SlopeCorrectionVel(VelData vel0, VelData* veld)
{
	//RotData rot;

	double gain1 = 0.6;//0.9
//	double gain2 = 0.3;
	int v_offset = 3;//弟SH2の場合(4,少し進む指令を入れると超信地旋回になる)、KAMUIの場合(0)

	// 上り坂または下り坂またはロール方向に車体が傾いている時に並進速度の補正を入れて，旋回時のトルク不足をごまかす（坂井 2012/5/3）
	if((abs(rot.pitch*180.0/3.14) > 3.0 || abs(rot.roll*180.0/3.14) > 5.0) && !(vel0.vel == 0 && vel0.avel == 0))
	{
		vel0.vel = vel0.vel + int((rot.pitch*180.0/3.14)*gain1) + v_offset;
		if(vel0.vel < v_offset && vel0.vel >= 0)
		{
			vel0.vel = v_offset;
		}
//		std::cout << "pitch:" << rot.pitch*180.0/3.14 << "(" << abs(rot.pitch*180.0/3.14) << ")" << " roll:" << rot.roll*180.0/3.14 << "(" << abs(rot.roll*180.0/3.14) << ")" << " add_vel : " << int((rot.pitch*180.0/3.14)*gain1) + v_offset << std::endl;
		std::cout << " vel(add_vel) : " << vel0.vel << "(" << int((rot.pitch*180.0/3.14)*gain1) + v_offset << ")" << std::endl;
	}

	/*
	// 左旋回時に後方へ動いてしまうことへの対処として，前進速度を加える（室 2012/5/3）
	if(vel0.vel == 0 && vel0.avel > 0)
	{
		vel0.vel = 1;
	}
	*/

	//フリッパーの対応
	// 一定角以上は上れないため後退する
	if(rot.pitch*180/3.14>= 40)
	{
		vel0.vel = -20;
		vel0.avel = 0;
	}
	//return vel0;
	//veld->vel = vel0.vel;
	//veld->avel = vel0.avel;
	*veld = vel0;
	//ROS_INFO("%d %d", veld.vel, veld.avel);//debug -> OK
}

//=======================================================================================================//
/*!
*	@brief 制御入力をセットする
*	@param veld veld0.velが動かしたい並進速度，veld0.avelが動かしたい角速度を意味
*/
//=======================================================================================================//
void KamuiControl::ControlFromVelData(VelData veld0, int16_t* data1, int16_t* data2)
{
	//WORD data1,data2;	 //マイコン送信データ
	//uint16_t data1,data2;	 //マイコン送信データ
	double wR_d=0,wL_d=0;//
    int sign = 0;

	VelData veld = veld0;
	
	//ROS_INFO("%d %d", veld.vel, veld.avel);//debug

	//不感帯への対応
	if(abs(veld.vel) > 5)
	{
		sign = (veld.vel > 0)?1:-1;
		veld.vel = sign * (int)((100-VEL_THRESHOLD)*((double)abs(veld.vel)/100.0)+VEL_THRESHOLD);
	}
	if(abs(veld.avel) > 5)
	{
		sign = (veld.avel > 0)?1:-1;
		veld.avel = sign * (int)((100-AVEL_THRESHOLD)*((double)abs(veld.avel)/100.0)+AVEL_THRESHOLD);
	}

	/*クローラー部分----------------------------------*/
	// 走行速度に倍率をかける
	if(veld.times==1)
	{
		wR_d = -( -veld.times*(double)veld.vel/100.0  -veld.times*2*(double)veld.avel/100.0 ) * CRAWLERLIMIT;
		wL_d =  ( -veld.times*(double)veld.vel/100.0  +veld.times*2*(double)veld.avel/100.0 ) * CRAWLERLIMIT;
	}
	else if(veld.times==2)
	{
		wR_d = -( -veld.times*(double)veld.vel/100.0  -veld.times*2*(double)veld.avel/100.0 ) * CRAWLERLIMIT;
        wL_d =  ( -veld.times*(double)veld.vel/100.0  +veld.times*2*(double)veld.avel/100.0 ) * CRAWLERLIMIT;
	}
	else
	{
		wR_d = -( -(double)veld.vel/100.0  -(double)veld.avel/100.0 ) * CRAWLERLIMIT ;
        wL_d =  ( -(double)veld.vel/100.0  +(double)veld.avel/100.0 ) * CRAWLERLIMIT;
	}

	// 目標値範囲のチェック
	if(wR_d >  CRAWLERFULL) wR_d =  CRAWLERFULL;
	if(wR_d < -CRAWLERFULL) wR_d = -CRAWLERFULL;
	if(wL_d >  CRAWLERFULL) wL_d =  CRAWLERFULL;
	if(wL_d < -CRAWLERFULL) wL_d = -CRAWLERFULL;

	// 目標値を16bitに変換
	*data1 = (short)(-wR_d / CRAWLERFULL * 32767.0);	// 右
	*data2 = (short)(-wL_d / CRAWLERFULL * 32767.0);	// 左

	//ROS_INFO("%d %d", data1, data2);//debug

	/*フリッパー--------------------------------------*/
	/*
	short flipper_speed = 0x0800;
	short right_flipper=0, left_flipper=0;

	//右フリッパーの設定
	if(veld.fvelright > 0)
	{
		right_flipper = veld.times*flipper_speed;
	}
	else if(veld.fvelright < 0)
	{
		right_flipper = -veld.times*flipper_speed;
	}
	else
	{
		right_flipper = 0;
	}
	//左フリッパーの設定
	if(veld.fvelleft > 0)
	{
		left_flipper = -veld.times*flipper_speed;
	}
	else if(veld.fvelleft < 0)
	{
		left_flipper = veld.times*flipper_speed;
	}
	else
	{
		left_flipper = 0;
	}
	*/

	////データ送信
	////ControlRobot(data1,data2,right_flipper,left_flipper);
	//ControlRobot(data2,data1,left_flipper,right_flipper);	//前進速度が与えられた時前進できるように修正した　李/2013.4.12
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kamui_control_node");

	KamuiControl kamuiCtrl;

	ROS_INFO("Kamui Control Start");

	ros::spin();

	return 0;
}
