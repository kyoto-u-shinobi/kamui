#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "sh2interface/sh2_tx.h"
#include <iostream>
#include <cmath>

#define DUTY_STEP	0.01

std::string state = "Waiting";
bool send_flag = true;

void state_Callback(const std_msgs::String::ConstPtr& state_msg)
{
	state = state_msg -> data;
	send_flag = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "led_control_node");

	ros::NodeHandle n;
	sh2interface::sh2_tx txdata;


	// subscriber and publisher
	ros::Subscriber sub = n.subscribe("state", 1000, state_Callback);
	ros::Publisher pub = n.advertise<sh2interface::sh2_tx>("sh2_tx", 1000);
	
	ros::Rate loop_rate(100);	// ループ頻度を設定(100 Hz)

	txdata.command = 0xb0;	// command
	txdata.id = 0x21;		// id

	double duty1 = 0; // R
	double duty2 = M_PI/3; // G
	double duty3 = M_PI*2/3; // B

	while (ros::ok())
	{
		if(state == "Normal" || state == "Following")	// Normal
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Tracking")	// Tracking
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0xae;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Rotating")	// Rotating
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0xae;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Closing")		// Closing
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Scanning")	// Scanning
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "GetInfo")	// GetInfo
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0x66;		// interval
		}
		else if(state == "Leaving")	// Leaving
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Stop_temp")	// Stop_temp
		{	
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0x66;	// interval
		}
		else if(state == "Waiting")	// Waiting
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0x66;	// interval
		}
		else if(state == "Moving")	// Moving
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0;	// interval
		}
		else if(state == "Teleope")	// Teleope
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0x4c;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0x33;	// interval
		}
		else if(state == "Finish")	// Finish
		{
			txdata.data[0] = (uint8_t)(0xfe * sin(duty1) * sin(duty1));	// red
			txdata.data[1] = (uint8_t)(0xfe * sin(duty2) * sin(duty2));	// green
			txdata.data[2] = (uint8_t)(0xfe * sin(duty3) * sin(duty3));	// blue
			txdata.data[3] = 0;	// interval

			duty1 += DUTY_STEP;	if(duty1 > 2*M_PI)	duty1 = 0;
			duty2 += DUTY_STEP;	if(duty2 > 2*M_PI)	duty2 = 0;
			duty3 += DUTY_STEP;	if(duty3 > 2*M_PI)	duty3 = 0;
		}
		else
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}

		if(send_flag || state == "Finish")
		{
			pub.publish(txdata);
			send_flag = false;
		}

		

		ros::spinOnce();

		loop_rate.sleep();
	}

	//TODO: 終了時に消灯
	

	return 0;
}
