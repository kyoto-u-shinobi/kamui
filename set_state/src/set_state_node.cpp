#include "ros/ros.h"
#include "set_state/pubState.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

std_msgs::String state0;

bool pub_state(set_state::pubState::Request  &req,
		 	   set_state::pubState::Response &res)
{
	state0.data = req.state;
	ROS_INFO("called set state: %s", state0.data.c_str());//debug
	
	return true;
}

void endflagCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) state0.data = "Waiting";
	ROS_INFO("Explorer Finish!!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "set_state_node");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("pubState", pub_state);
	ros::Publisher pub = n.advertise<std_msgs::String>("state", 1000);
	ros::Subscriber explorer_end_flag_sub = n.subscribe("explorer_end_flag", 1000, endflagCallback);
  
	ros::Rate loop_rate(100);	// ループ頻度を設定(100 Hz)

	state0.data = "Waiting";	// デフォルトは"Waiting"

	while (ros::ok())
	{
		pub.publish(state0);
		//ROS_INFO("state: %d", state0.data);//debug

		ros::spinOnce();

		loop_rate.sleep();
	}

	ros::spin();
	
  return 0;
}
