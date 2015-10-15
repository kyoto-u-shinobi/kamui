/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Gonçalo Cabrita on 01/03/2012
 *********************************************************************/

#include <iostream>

#include <ros/ros.h>
#include <can_msgs/CANFrame.h>

#include "CANUSB.h"

CANUSB * can_usb_adapter;

ros::Publisher rx_pub;

void CANFrameToSend(const can_msgs::CANFrame::ConstPtr& msg)
{
    char buffer[5];
    
    sprintf(buffer, "%03X%d", msg->cob_id, (int)msg->data.size());
   
    std::string frame = buffer;
    for(int i=0 ; i<msg->data.size() ; i++)
    {
        sprintf(buffer, "%02X", msg->data[i]);
        frame.append(buffer);
    }

    ROS_INFO("CANUSB - %s - Sending CAN Frame: %s", __FUNCTION__, frame.c_str());
    
	can_usb_adapter->transmitStandardFrame(&frame);
}

void CANFrameReceived(std::string * frame)
{
    ROS_INFO("CANUSB - %s - Received CAN Frame: %s", __FUNCTION__, frame->c_str());
    
    char cob_id_string[4];
    frame->copy(cob_id_string, 3, 1);
    int cob_id;
    sscanf(cob_id_string, "%X", &cob_id);
    
    ROS_INFO("CANUSB - %s - COB ID %s %d", __FUNCTION__, cob_id_string, cob_id);

    int data_size = (frame->length()-5)/2;
    
    can_msgs::CANFrame frame_msg;
    frame_msg.stamp = ros::Time::now();
    frame_msg.cob_id = cob_id;
    
    char buffer[3];
    int byte;
    frame_msg.data.resize(data_size);
    for(int i=0 ; i<data_size ; i++)
    {
        frame->copy(buffer, 2, 5+i*2);
        sscanf(buffer, "%X", &byte);
        frame_msg.data[i] = byte;
    }
    
    rx_pub.publish(frame_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "CANUSB_node");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	rx_pub = n.advertise<can_msgs::CANFrame>("/can_bus_rx", 10);
    ros::Subscriber tx_sub = n.subscribe<can_msgs::CANFrame>("/can_bus_tx", 10, CANFrameToSend);
	
	std::string port;
	pn.param<std::string>("port", port, "/dev/tty.USB0");
    int bit_rate;
	pn.param("bit_rate", bit_rate, 5);
    
    can_usb_adapter = new CANUSB(&port, &CANFrameReceived);
    
    ROS_INFO("CANUSB -- Opening CAN bus...");
    if( !can_usb_adapter->openCANBus(bit_rate) )
    {
        ROS_FATAL("CANUSB -- Failed to open the CAN bus!");
		ROS_BREAK();
    }
    ROS_INFO("CANUSB -- The CAN bus is now open!");
    
	ros::spin();
    
    delete can_usb_adapter;
    
  	return(0);
}

// EOF

