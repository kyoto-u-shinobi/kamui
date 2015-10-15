#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include "wall_follower/walls.h"

//各種定義
#define	ROAD_WIDTH	850		//!< 道の幅[m]//900mm...(JapanOpen),1200mm(WorldCup)

//☆ロボットの筐体，自律走行のパラメータ
#define	ROBOT_WIDTH 350				/*横幅*/

ros::Publisher walls_pub;//

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

Distance dist, dist2;

//-----------------------------------------------------
// URGデータのコールバック関数
// 計測距離データをロボット座標での3次元点データに変換したものから
// ロボット周囲の障害物までの距離を計算する
//-----------------------------------------------------
void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	double urg_x, urg_y;
	wall_follower::walls walls;

	dist.Clear();
	
	for(int i = 0; i < (int)msg->points.size(); i++){
		urg_x = msg->points[i].x * 1000;	// [m] から [mm] に変換
		urg_y = msg->points[i].y * 1000;	// [m] から [mm] に変換
		
		if(urg_x >= 100){
			//前方
			if( (urg_y > -(ROBOT_WIDTH/2+10)) && (urg_y < +(ROBOT_WIDTH/2+10)) )
			{
				if(urg_x < dist.front)
				{
					dist.front = (int)urg_x;
				}
			}
			if(urg_x <= ROAD_WIDTH/2){
				//右前の方向
				if ((-urg_y > 21) && (-urg_y < dist.right) ) 
				{
					dist.right = -(int)urg_y;
				}
				//左前の方向
				if( (urg_y > 21) && (urg_y < dist.left)) 
				{
					dist.left = (int)urg_y;
				}
			}
		}
		else if(urg_x < 100 && urg_x >= -ROBOT_WIDTH/2+100)//後方のデータ
		{
			//右後ろの方向
			if ( (-urg_y > 21) && (-urg_y < dist.right2)) 
			{
				dist.right2 = -(int)urg_y;
			}
			//左後ろの方向
			if ( (urg_y > 21) && (urg_y < dist.left2)) 
			{
				dist.left2 = (int)urg_y;
			}
		}
	}
    //ROS_INFO("F=%4.0f  L=%4.0f  L2=%4.0f  R=%4.0f  R2=%4.0f", dist.front,  dist.left, dist.left2, dist.right, dist.right2);

    // 水平LRFとジンバルLRFのうち小さい方を採用
    walls.front = (dist.front < dist2.front) ? dist.front : dist2.front;
    walls.left = (dist.left < dist2.left) ? dist.left : dist2.left;
    walls.left2 = (dist.left2 < dist2.left2) ? dist.left2 : dist2.left2;
    walls.right = (dist.right < dist2.right) ? dist.right : dist2.right;
    walls.right2 = (dist.right2 < dist2.right2) ? dist.right2 : dist2.right2;

    ROS_INFO("F=%4.0f  L=%4.0f  L2=%4.0f  R=%4.0f  R2=%4.0f", walls.front,  walls.left, walls.left2, walls.right, walls.right2);


	walls_pub.publish(walls);
}

// 3Dマップから得たwallsデータ
void wallsCallback(const wall_follower::walls::ConstPtr& msg)
{
    dist2.Clear();

    dist2.front = msg->front;
	dist2.left = msg->left;
	dist2.left2 = msg->left2;
	dist2.right = msg->right;
	dist2.right2 = msg->right2;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detect_near_wall_node2");

	ros::NodeHandle n;

    ros::Subscriber urg_sub = n.subscribe("cloud", 1000, cloudCallback);
    ros::Subscriber walls_sub = n.subscribe("gimbal_walls", 1000, wallsCallback);
	walls_pub = n.advertise<wall_follower::walls>("walls", 50);

	ros::spin();
}
