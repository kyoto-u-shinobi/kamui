#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Image.h"

// グローバル変数設定
double Thermo[60][80];	// 温度データの配列
			// Thermo[行][列]ではなく[列][行]なのはOpenCVと統一するため
sensor_msgs::Image::ConstPtr image;

bool pub_flag;//配信フラグ
bool subs_flag;//購読フラグ
//=============================================
//@name TIconvert
//@brief 熱画像から温度を抜きだしThermo配列に代入する
//@data 2013/10/21
//@attention 温度データは１点飛ばしで取得する
//=============================================	
void TIconvert()
{
  // TODO:要作成
  // imageから温度データを抜き出してThermo配列に代入する
  // 温度[deg]への変換、90度回転、1点飛ばしの3つの処理が必要

//原点は左上。横軸はx軸、縦軸はy軸

  int width  = image->width;//（本来の）横方向のセル数
  int height = image->height;//（本来の）横方向のセル数
  int step   = image->step;
 


  //PI１６０からの生データを格納
  double RawThermalData[step][height];//変換したデータを格納

  for (int j = 0; j < height; j++)
  {
    for(int i = 0; i < step; i++)
    {
      RawThermalData[i][j] = image->data[j*step+i];
      //↑ここで縦横を間違えている可能性がある
      //やってみて挙動がおかしければまず疑うこと
 //     ROS_INFO("%d %d",i,j);
    }
  }

  //温度を計算
  double PreThermalData[width][height];
  for (int j = 0; j < height; j++)
  {
    for(int i = 0; i < width; i++)
    {
       PreThermalData[i][j] = ((double)(RawThermalData[i*2][j]+RawThermalData[i*2+1][j]*256)-1000)/10;
    }
  }
  

  //回転
 double RotThermalData[height][width];
  for (int j = 0; j < width; j++)
  {
    for(int i = 0; i < height; i++)
    {
       RotThermalData[i][j] = PreThermalData[j][height-i];
    }
  }
  //一つ飛ばしのデータ
  for (int j = 0; j < 80; j++){	// jは行のループ
    for (int i = 0; i < 60; i++){	// iは列のループ
    Thermo[i][j] = RotThermalData[i*2][j*2];
    }
  }
//	ROS_INFO("Thermo[30][40]=%f",Thermo[30][40]);
}

//=============================================
//@name thermoCallback
//@brief msgで受け取った熱画像を格納する
//@data 2013/10/21
//@attention
//=============================================	
void Thermal_image_Callback(const sensor_msgs::Image::ConstPtr& thermal_image)
{
  if(subs_flag == true){
  // 取得を示すROS_INFOを流す
//  ROS_INFO("Now get thermal_image");
	
  // 取得したthermal_imageを変数に格納する
  image = thermal_image;


  // 配信準備ができたら熱画像を温度データに変換する関数を呼び出さないようにする
          TIconvert();
          subs_flag = false;
          pub_flag = true;
 // 	  ROS_INFO("Now convert thermal_image");
  }
}

//=============================================
//@name main
//@brief メインループ
//@data 2013/10/21
//@attention
//=============================================	
int main(int argc, char **argv)
{
	// ROSの初期設定
	ros::init(argc, argv, "thermo_image_conv_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);	// ループの待機時間(Hz)

	// 熱画像受信用のサブスクライバーの定義
	ros::Subscriber thermal_image_sub = n.subscribe("thermal_image", 100, Thermal_image_Callback);

	// 温度データ配信用のパブリッシャーの定義
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("Thermo", 100);

	// 配信する配列のmsgを定義
	std_msgs::Float32MultiArray thermo_msg;
	
	subs_flag == true;
	// メインループ
	while (ros::ok())
	{
		// msg配列の初期化
		thermo_msg.data.clear();

		// コールバックを呼び出して熱画像の取得
		ros::spinOnce();

		// 熱画像を温度データに変換する関数の呼び出し

		//TIconvert();//はここにいれてはいけない
		//Callbackがループしている間に関数を呼び出すとpublishの部分が実行されないのでエラーになる
		//下記のサイトを参照(英語)
	//http://answers.ros.org/question/11745/publisher-and-subscriber-together-problem/



		// msg配列にデータを入れるループ
		// 注釈：配列でmsgを送るときは
		//	{msg名}.data.push_back({値})
		//	を使ってmsg配列の最後尾にデータを付け足していく形でmsgを作る
		//	そのため、多次元配列の場合、forループで何ちゃって多次元配列msgを作る
		for (int j = 0; j < 80; j++){	// jは行のループ
			for (int i = 0; i < 60; i++)	// iは列のループ
			{
				// 温度データをmsg配列に代入
				// 注釈：thermoデータの並びは
				//	(1,1)→(1,2)→・・・→(1,60)→(2,1)→・・・
				//	にしてください
				//	(熱画像の左上(1,1),右上(1,60),左下(80,1),右下(80,60))
				thermo_msg.data.push_back(Thermo[i][j]);
			}
		}
		pub_flag = true;
		// 配信準備ができたらmsg配列を配信する
		if(pub_flag == true){
			pub.publish(thermo_msg);
                	pub_flag = false;
			subs_flag = true;
		}
		// ROS_INFOで配信していることを伝える
		//ROS_INFO("I published something!");
		ros::spinOnce();

		// 次のデータを送る前にloop_rateだけ待機
		loop_rate.sleep();
	}
	return 0;
}
