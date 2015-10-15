#include "ros/ros.h"
#include "sh2interface/encoder.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include "calc_odometry/posdata.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

//ロボットに関する定数
#define Rw  0.095					/*! 車輪半径[m] (直径0.19m)*/
#define T  0.230					/*!トレッド[m]             */

#define CRAWLERFULL  12.0			/*! クローラの構造上の最大回転速度[rad/s]*/
#define CRAWLERLIMIT 3.0			/*! クローラの制限最大回転速度[rad/s]    */

//オドメトリに関する定数
#define SETTING_ANGLE_PITCH 1.1		/*! ICのpitch取り付け角[deg]      */
#define THRESHOLD_PITCH 0.0			/*! 坂と判定するpitch閾値[deg]    */

#define SETTING_ANGLE_ROLL 0.0		/*! ICのroll取り付け角[deg]      */
#define THRESHOLD_ROLL 0.0			/*! roll角度反映の閾値[deg]　　  */

#define VEL_THRESHOLD 5			/*! 並進速度に関する不感帯 */
#define AVEL_THRESHOLD 5//19			/*! 角速度に関する不感帯   */

#define ADJUST_F 1.00//3.20//1.55				/*! 前進に関するエンコーダの係数 */
#define ADJUST_B 1.00//0.85//0.901			/*! 後進に関するエンコーダの係数   */
#define W_ADJUST 1.00				/*! 回転に関する係数             */

#define THRESH_ODD_SENSOR 0.001		/*!	オドメトリとIC3を切り替える閾値[rad]*/

#define PLUSE_PER_ROUND_C 136970.24       /*! クローラのエンコーダのパルス数[pulse/round]  */
#define PLUSE_PER_ROUND_F 136970.24*5.7	  /*! フリッパーのエンコーダのパルス数[pulse/round]*/

#define SIZE_DRIFT 20

#define SUBS_F 50 // rate of subscribing encoder data [Hz]


//==================================================================//
//エンコーダーデータクラス
//=========------===================================================//
class EncData{
public:
	short e_data[4];	//kamuiでは，0:左フリッパーエンコーダ値,1:右フリッパーエンコーダ値,2：左クローラエンコーダ値,3:右クローラエンコーダ値,
						//kohga3では，0:左フリッパーポテンショ,1:右フリッパーポテンショ,2：左クローラエンコーダ,3:右クローラエンコーダ値,
	uint8_t right_id;		//
	uint8_t left_id;		//
	uint8_t right_status;	//
	uint8_t left_status;	//

	//コンストラクタ&デストラクタ
	EncData(){
		ros::NodeHandle n;
		enc_sub = n.subscribe("encoder", 1000, &EncData::encCallback, this);
		Clear();
	}
	~EncData(){}
	void Clear(void){
        e_data[0] = 0;
        e_data[1] = 0;
        e_data[2] = 0;
        e_data[3] = 0;
	}
	
	//オペレータのオーバーロード
	EncData operator=(EncData &e1){
		e_data[0] = e1.e_data[0];
		e_data[1] = e1.e_data[1];
		e_data[2] = e1.e_data[2];
		e_data[3] = e1.e_data[3];
		return *this;
	};

	void encCallback(const sh2interface::encoder::ConstPtr& msg)
	{
		e_data[0] = msg->e_data[0];
		e_data[1] = msg->e_data[1];
		e_data[2] = msg->e_data[2];
		e_data[3] = msg->e_data[3];
	}

protected:
	ros::Subscriber enc_sub;
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
	geometry_msgs::Quaternion q;	//クォータニオン
	geometry_msgs::Vector3 ang_vel;	//角速度[rad/s]

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
		//q = msg->orientation;
		tf::Quaternion q0, qi;
		// amuから姿勢データ受信
		tf::quaternionMsgToTF(msg->orientation, q0); // Quaternion msgからtf::Quaternionに変換
		qi = q0;//tf::inverse(q0);
		tf::Matrix3x3(qi).getRPY(roll, pitch, yaw);	// tf::Quaternionからroll, pitch, yawを取得
		ROS_INFO("RPY = (%lf, %lf, %lf)", roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);

		tf::quaternionTFToMsg(qi, q);
		ang_vel = msg->angular_velocity;
	}
	
protected:
	ros::Subscriber pose_sub;
};

//==================================================================//
//位置データクラス
//=========------===================================================//
class OrthData
{
public:
	double x;		//x[m]	
	double y;		//y[m]
	double z;		//z[m]

	OrthData(){Clear();};
	~OrthData(){};

	virtual void Clear()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
	OrthData operator=(OrthData &orth0){
		x  = orth0.x;
		y  = orth0.y;
		z  = orth0.z;
		return *this;
	}
};

//==================================================================//
//フリッパー角度データクラス
//=========------===================================================//
class FlipperData
{
public:
	double flipperFR;//右フリッパーの角度[rad]
	double flipperFL;//左フリッパーの角度[rad]
	double flipperRR;//右フリッパーの角度[rad]
	double flipperRL;//左フリッパーの角度[rad]

	FlipperData(){Clear();};
	~FlipperData(){Clear();};

	virtual void Clear()
	{
		flipperFR = 0;
		flipperFL = 0;
		flipperRR = 0;
		flipperRL = 0;
	}
	FlipperData operator=(FlipperData &fdata)
	{
		flipperFR = 0;
		flipperFL = 0;
		flipperRR = 0;
		flipperRL = 0;
		return *this;
	}
};

//==================================================================//
// PosDataクラス
//==================================================================//
class PosData : virtual public OrthData,virtual public RotData,virtual public FlipperData
{
public:
	//コンストラクタ＆デストラクタ
	PosData(){
		Clear();
	}
	~PosData(){}
	void Clear(void){
		OrthData::Clear();
		RotData::Clear();
		FlipperData::Clear();
	}

	//オペレータのオーバーロード
	PosData operator=(PosData &pos1){
		x = pos1.x;
		y = pos1.y;
		z = pos1.z;
		roll  = pos1.roll;
		pitch = pos1.pitch;
		yaw   = pos1.yaw;
		flipperFR = pos1.flipperFR;
		flipperFL = pos1.flipperFL;
		flipperRR = pos1.flipperRR;
		flipperRL = pos1.flipperRL;
		return *this;
	};

	//2Dでの2点間の距離を返す
	double CalcDistance(PosData pos0)
	{
		return sqrt( pow((x-pos0.x),2.0)+pow((y-pos0.y),2.0)); 
	}
	//前のスキャン位置からの角度を返す
	double CalcRotation(PosData pos0)
	{
		return (abs(yaw-pos0.yaw) > (2*3.1416-abs(yaw-pos0.yaw)))?2*3.1416-abs(yaw-pos0.yaw):abs(yaw-pos0.yaw);
	}
};

//=======================================================================================================//
/*! 
	@class	KAMUI制御用クラス
*/
//=======================================================================================================//
class mykamui
{
public:
	EncData encdata;	 //!< マイコンからの受信データ管理用
	PosData curpos;		 //!< オドメトリによる現在位置
	PosData curvel;		 //!< オドメトリによる現在速度

	//--------------コンストラクタ＆デストラクタ---------------
	mykamui(){};					//!< デフォルトコンストラクタ
	~mykamui(){};					//!< デストラクタ

	//------------------------関数-----------------------------
	//オドメトリ系
	bool CalcOdometry(void);					//!< オドメトリを計算する
};

//=======================================================================================================//
//名前：CalcOddmetory
//説明：オドメトリを計算
/*!
	@param なし
	@retval 現状trueしか返さない．
*/
//=======================================================================================================//
bool mykamui::CalcOdometry(void)
{
	/*ローカル変数------------------------*/
	static bool flag_init = false;								//初期化ずみか否か
	static double roll_pre = 0, pitch_pre = 0, yaw_pre = 0;		//1ステップ前の姿勢情報管理用
	short encdata_cur[4];										//現在のエンコーダデータ
	short encdata_pre[4];//static EncData encdata_pre;			//前回のエンコーダデータ

	double roll_ic = 0, pitch_ic = 0, yaw_ic = 0;				//現在の姿勢情報保存用
	double roll_tokin = 0, pitch_tokin = 0, yaw_tokin = 0;		//現在の姿勢情報保存用

	RotData rot_ic;//,rot_tokin;

	double wr = 0,wl = 0,v = 0,w = 0;							//左右の速度，並進・回転速度
	double wr_xy = 0,wl_xy = 0,vr_xy = 0,vl_xy = 0,v_xy=0,v_z = 0,w_xy = 0,tread = 0;
	double w_sensor = 0;
	double roll=0,pitch=0,yaw =0;

	double flipperRw = 0,flipperLw = 0;							//フリッパーの回転速度[rad/s]
	double delta_t = 1/(double)SUBS_F;//0.02;	//sampling_time;		//サンプリングタイム[ms]
	//bool flag=0;
	int flag=0;

	// エンコーダデータ取得
	encdata_cur[0] = encdata.e_data[0];
	encdata_cur[1] = encdata.e_data[1];
	encdata_cur[2] = encdata.e_data[2];
	encdata_cur[3] = encdata.e_data[3];
	
/*
	//姿勢センサから情報を取得
	flag = GetDataFromSensor(rot_ic);
*/
	yaw_ic   = rot_ic.yaw;
	roll_ic  = rot_ic.roll;
	pitch_ic = rot_ic.pitch;

	/*初回のみの処理-------------------------------*/
	if(flag_init == false){
		//１ステップ前
		roll_pre  = roll_ic;
		pitch_pre = pitch_ic;
		yaw_pre   = yaw_ic;
		flag_init = true;		//初期化済み
		ROS_INFO("flag_init: true");//debug
		//encdata_pre.Clear();
		encdata_pre[0] = 0;
		encdata_pre[1] = 0;
		encdata_pre[2] = 0;
		encdata_pre[3] = 0;
	}

	/*姿勢センサデータからノイズを削除-------------*/
	pitch = pitch_ic;
	roll  = roll_ic;
	yaw   = yaw_ic;
	
	//pitch(deg⇒radへの変換も同時に行う)
	if(abs(pitch)-abs(SETTING_ANGLE_PITCH) > THRESHOLD_PITCH)
	{
		pitch = (pitch - SETTING_ANGLE_PITCH)/180.0*M_PI;
	}
	else
	{
		pitch = 0;
	}
	//roll
	if(abs(roll) - abs(SETTING_ANGLE_ROLL) > THRESHOLD_ROLL)
	{
		roll = (roll - SETTING_ANGLE_ROLL)/180.0*M_PI;
	}
	else
	{
		roll = 0;
	}

	//yaw(deg⇒radへの変換も同時に行う)
	if (yaw < 180 && yaw > 90 &&   yaw_pre > -180 && yaw_pre < -90)
	{
        w_sensor  = (yaw - yaw_pre - 360.0)/180.0*M_PI/delta_t;
	}
	else if(yaw > -180 && yaw < -90 &&  yaw_pre < 180 && yaw_pre > 90)
	{
        w_sensor  = (yaw + 360.0 - yaw_pre)/180.0*M_PI/delta_t;   
	}
	else
	{
        w_sensor  = (yaw - yaw_pre)/180.0*M_PI/delta_t;    
	}

	/*エンコーダの値から明らかなノイズを削除--------*/
	
	if(abs(encdata_cur[0]) > 5000)encdata_cur[0] = encdata_pre[0];
	else encdata_pre[0] = encdata_cur[0];
	if(abs(encdata_cur[1]) > 5000)encdata_cur[1] = encdata_pre[1];
	else encdata_pre[1] = encdata_cur[1];	
	if(abs(encdata_cur[2]) > 10000)encdata_cur[2] = encdata_pre[2];
	else encdata_pre[2] = encdata.e_data[2];
	if(abs(encdata_cur[3]) > 10000)encdata_cur[3] = encdata_pre[3];
	else encdata_pre[3] = encdata_cur[3];	

	/*エンコーダの正負で係数を変更する*/
	if(encdata_pre[2] > 0)
	{
		wr = (((double)encdata_cur[2]/ (PLUSE_PER_ROUND_C / ADJUST_F) * 2*M_PI) / delta_t); //[rad/s]
	}else
	{
		wr = (((double)encdata_cur[2]/ (PLUSE_PER_ROUND_C / ADJUST_B) * 2*M_PI) / delta_t); //[rad/s]
	}
	if(encdata_pre[3] > 0)
	{
		wl = (((double)encdata_cur[3]/ (PLUSE_PER_ROUND_C / ADJUST_F) * 2*M_PI) / delta_t); //[rad/s]
	}
	else
	{
		wl = (((double)encdata_cur[3]/ (PLUSE_PER_ROUND_C / ADJUST_B) * 2*M_PI) / delta_t); //[rad/s]
	}


	//cout<<"FL："<<encdata.e_data[0]<<"FR:"<<encdata.e_data[1]<<"CL:"<<encdata.e_data[2]<<"CR:"<<encdata.e_data[3]<<endl;
	
	/*エンコーダデータから接地平面での速度を計算----*/
	v = Rw * (wr + wl) / 2.0;
	w = Rw * (wr - wl) / (T *  W_ADJUST);

	/*3次元空間での速度に分解-----------------------*/
	vr_xy = (wr)*cos(pitch);
	vl_xy = (wl)*cos(pitch);
	v_xy = v*cos(pitch);
	v_z  = v*sin(pitch);
	tread = T*cos(roll);
	w_xy = Rw * (vr_xy -vl_xy)/(tread*W_ADJUST);   


	//IC3に接続できなかった場合
	if(abs(w_xy) <0.005)w_xy=0;
	if(flag == 0)w_sensor = w_xy;
    double w_trust = w_xy;
    //低速度域はオドメトリを信頼
	//if(abs(w_xy) < THRESH_ODD_SENSOR)
	//{
	//	w_trust = w_xy; 
	//}
	//else
	//{	
	//高速度域はICを信頼する
	//したがってIC3は高速度域でいい応答をするように設定する(低速度は関係なし!)
	w_trust = w_sensor;
	//}
	//if(abs(w_sensor) > 2)
	//{
	//	w_trust = w_xy;
	//}

	/*現在位置を計算-------------------------------*/
	curpos.x     = curpos.x + v_xy*delta_t*cos(curpos.yaw + w_trust*delta_t/2);    //中間姿勢に変更
	curpos.y     = curpos.y + v_xy*delta_t*sin(curpos.yaw + w_trust*delta_t/2);    //中間姿勢に変更
	curpos.yaw   = curpos.yaw + w_trust*delta_t;
	curpos.z     = curpos.z + v_z*delta_t;
	curpos.pitch = pitch;
	curpos.roll  = roll;

	/*現在速度を計算-------------------------------*/
	curvel.x     = v_xy*cos(curpos.yaw + w_trust*delta_t/2);    //中間姿勢に変更
	curvel.y     = v_xy*sin(curpos.yaw + w_trust*delta_t/2);    //中間姿勢に変更
	curvel.z     = v_z;
	curvel.yaw   = w_trust;	
	//curvel.pitch = pitch;
	//curvel.roll  = roll;

	//角速度算出用に一つ前の角度を保存---------------
	yaw_pre   = yaw_ic;
	pitch_pre = pitch_ic;
	roll_pre  = roll_ic;

	/*フリッパー角度の計算--------------------------*///TODO: フリッパーの角度おかしい
	flipperRw = (((double)encdata_cur[0]/ (PLUSE_PER_ROUND_F) * 2*M_PI) / delta_t); //[rad/s];
	flipperLw = (((double)encdata_cur[1]/ (PLUSE_PER_ROUND_F) * 2*M_PI) / delta_t); //[rad/s];
	curpos.flipperFR = curpos.flipperFR - flipperRw * delta_t;	//[rad]
	curpos.flipperFL = curpos.flipperFL + flipperLw * delta_t;	//[rad]
	curvel.flipperFR = -flipperRw;	//[rad]
	curvel.flipperFL = flipperLw;	//[rad]

	//ROS_INFO("pre:%d %d %d %d", encdata_pre[0], encdata_pre[1], encdata_pre[2], encdata_pre[3]);//debug
	//ROS_INFO("cur:%d %d %d %d", encdata_cur[0], encdata_cur[1], encdata_cur[2], encdata_cur[3]);//debug

	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calc_odometry_node");

	ros::NodeHandle n;

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	ros::Publisher pos_pub = n.advertise<calc_odometry::posdata>("robot_pos", 1000);
	
	mykamui kamui;
	nav_msgs::Odometry odom;
	calc_odometry::posdata pos;

	ros::Rate loop_rate(SUBS_F);

	while (ros::ok())
	{
		//オドメトリ計算
		kamui.CalcOdometry();
		
		//オドメトリデータセット
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";
		odom.pose.pose.position.x = kamui.curpos.x;
		odom.pose.pose.position.y = kamui.curpos.y;
		odom.pose.pose.position.z = kamui.curpos.z;
		odom.pose.pose.orientation = kamui.curpos.q;
		//odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(kamui.curpos.roll, kamui.curpos.pitch, kamui.curpos.yaw);

		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = kamui.curvel.x;
		odom.twist.twist.linear.y = kamui.curvel.y;
		odom.twist.twist.linear.z = kamui.curvel.z;
		odom.twist.twist.angular = kamui.curpos.ang_vel; //amuそのまま
		
		
		pos.x = kamui.curpos.x;
		pos.y = kamui.curpos.y;
		pos.z = kamui.curpos.z;
		pos.roll = kamui.curpos.roll;
		pos.pitch = kamui.curpos.pitch;
		pos.yaw = kamui.curpos.yaw;
		pos.flipperFR = kamui.curpos.flipperFR;
		pos.flipperFL = kamui.curpos.flipperFL;


		odom_pub.publish(odom);
		pos_pub.publish(pos);
		//ROS_INFO("%f %f %f", kamui.curpos.x, kamui.curpos.y, kamui.curpos.z);//debug

		ros::spinOnce();

		loop_rate.sleep();
	}
}
