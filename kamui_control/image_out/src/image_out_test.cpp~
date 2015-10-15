//=================================================//
/*! @file
 @brief 画像保存のノードのテストノード
 @author D.Furutani
 @date 2013/11/15
 @attention 画像を流すだけ

 @TODO 
*/
//=================================================//

#include <ros/ros.h>
#include <string>
#include "image_out/msg2file.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 配信用のファイル名
std::string filename( "/home/furutani/IMAGEFILE.png" );

//=============================================
//@name main
//@brief メインループ
//@data 2013/11/15
//@attention
//=============================================
int main(int argc, char** argv)
{
  
  // ROSの初期設定
  ros::init(argc, argv, "image_out_test");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(1);	// ループの待機時間(Hz)

  // パブリッシャーの立ち上げ
  image_transport::ImageTransport it_(nh_);
  image_transport::Publisher image_pub_;
  image_pub_ = it_.advertise("Imagemsg2file", 5);
  
  // OpenCV用変数の定義
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv::Mat M_img;

  // srvのクライアントの立ち上げ
  ros::ServiceClient client = nh_.serviceClient<image_out::msg2file>("msg2file_srv");
  image_out::msg2file srv;
  srv.request.filename = "IMAGEFILE.png";
  srv.request.directoryname = "/home/furutani/image/";
  srv.request.savemode = 0;

  // 画像の読み込み
  M_img = cv::imread(filename, 1);

  // cv_bridgeの変数に値を代入
  cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
  cv_ptr->image    = M_img;

  // 画像保存srvの呼び出し
  // 注：srv実行後に保存したいimageをトピックに流すこと
  if (client.call(srv))
  {
    image_pub_.publish(cv_ptr->toImageMsg());
    ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

  // テストプログラムは一度画像を流したらそれで終了して何もしない
  while(ros::ok())
  {
    loop_rate.sleep();
  }
  return 0;
}
