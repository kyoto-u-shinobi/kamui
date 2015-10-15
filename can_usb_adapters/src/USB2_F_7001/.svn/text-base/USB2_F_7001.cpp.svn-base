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
* Author: Gonçalo Cabrita on 01/03/2010
*********************************************************************/
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

#include "USB2_F_7001.h"

#include <ros/ros.h>

USB2_F_7001::USB2_F_7001(std::string * serial_port_name, boost::function<void(std::string*)> f) : serial_port_()
{
    open_ = false;
    
    try{ serial_port_.open((char*)serial_port_name->c_str(), 115200); }
	catch(cereal::Exception& e)
	{
		ROS_FATAL("USB2_F_7001 - %s - Failed to open the serial port!!!", __FUNCTION__);
        ROS_BREAK();
	}
    
    receivedFrameCallback = f;
}

USB2_F_7001::~USB2_F_7001()
{
    closeCANBus();
    
	serial_port_.close();
}

bool USB2_F_7001::openCANBus(USB2_F_7001_BitRate bit_rate)
{
    if(open_) return false;
    
    // Set the desired bit rate
    if( !setCANBitRate(bit_rate) )
    {
        ROS_ERROR("USB2_F_7001 - %s - Failed to set bit rate!", __FUNCTION__);
        return false;
    }
    
    char msg[3];
    
    msg[0] = 'O';
    msg[1] = USB2_F_7001_OK;
    msg[2] = 0;
    
    // Open the CAN bus
    try{ serial_port_.write(msg); }
    catch(cereal::Exception& e)
    {
        ROS_ERROR("USB2_F_7001 - %s - Failed to send open message!", __FUNCTION__);
        return false;
    }
    
    // Reply
    char reply;
    serial_port_.read(&reply, 1);
    if(reply != USB2_F_7001_OK) return false;
    
    open_ = true;
    
    // Clear the buffers
    /*if( !clearBuffers() )
    {
        ROS_ERROR("USB2_F_7001 - %s - Failed to clear the buffers!", __FUNCTION__);
        return false;
    }*/
    
    // Start streaming data
    if( !serial_port_.startReadBetweenStream(boost::bind(&USB2_F_7001::newFrameCallback, this, _1), 't', USB2_F_7001_OK) )
	{
        closeCANBus();
        ROS_ERROR("USB2_F_7001 - %s - Failed to start streaming data!", __FUNCTION__);
		return false;
	}
    
    return true;
}

bool USB2_F_7001::closeCANBus()
{
    if(!open_) return false;
    
    serial_port_.stopStream();
    
    char msg[3];
    
    msg[0] = 'C';
    msg[1] = USB2_F_7001_OK;
    msg[2] = 0;
    
    try{ serial_port_.write(msg); }
    catch(cereal::Exception& e)
    {
        ROS_ERROR("USB2_F_7001 - %s - Failed to send close msg!", __FUNCTION__);
        return false;
    }
    
    // Reply
    char reply;
    serial_port_.read(&reply, 1);
    if(reply == USB2_F_7001_OK)
    {
        open_ = false;
        return true;
    }
    
    ROS_ERROR("USB2_F_7001 - %s - Did not get reply - %d", __FUNCTION__, (int)reply);
    return false;
}

bool USB2_F_7001::reset()
{
    if(open_) return false;
    
    char msg[3];
    
    msg[0] = 'R';
    msg[1] = USB2_F_7001_OK;
    msg[2] = 0;
    
    try{ serial_port_.write(msg); }
    catch(cereal::Exception& e)
    {
        ROS_ERROR("USB2_F_7001 - %s - Failed to send reset msg!", __FUNCTION__);
        return false;
    }
    
    // Reply
    char reply;
    serial_port_.read(&reply, 1);
    if(reply == USB2_F_7001_OK) return true;
    
    ROS_ERROR("USB2_F_7001 - %s - Did not get reply - %d", __FUNCTION__, (int)reply);
    return false;
}

bool USB2_F_7001::setCANBitRate(USB2_F_7001_BitRate bit_rate)
{
    if(open_) return false;
    
    char msg[4];
    
    msg[0] = 'S';
    msg[1] = bit_rate+48;
    msg[2] = USB2_F_7001_OK;
    msg[3] = 0;
    
    try{ serial_port_.write(msg); }
    catch(cereal::Exception& e)
    {
        ROS_INFO("USB2_F_7001 - %s - Unable to send bit rate!", __FUNCTION__);
        return false;
    }
    
    // Reply
    char reply;
    serial_port_.read(&reply, 1);
    if(reply == USB2_F_7001_OK) return true;
    
    ROS_ERROR("USB2_F_7001 - %s - Did not get reply - %d", __FUNCTION__, (int)reply);
    return false;
}

bool USB2_F_7001::clearBuffers()
{
    char msg[3];
    
    msg[0] = 'E';
    msg[1] = USB2_F_7001_OK;
    msg[2] = 0;
    
    try{ serial_port_.write(msg); }
    catch(cereal::Exception& e)
    {
        ROS_INFO("USB2_F_7001 - %s - Unable to send clear buffers msg!", __FUNCTION__);
        return false;
    }
    
    // Reply
    char reply;
    serial_port_.read(&reply, 1);
    if(reply == USB2_F_7001_OK) return true;
    
    ROS_ERROR("USB2_F_7001 - %s - Did not get reply - %d", __FUNCTION__, (int)reply);
    return false;
}

bool USB2_F_7001::transmitStandardFrame(std::string * frame)
{
    if(!open_) return false;
    
    char ok[2];
    ok[0] = USB2_F_7001_OK;
    ok[1] = 0;
    
    std::string msg;
    
    msg = "t";
    msg.append(*frame);
    msg.append(ok);
    
    serial_port_.pauseStream();
    
    try{ serial_port_.write(msg.c_str()); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    
    // Reply
    char reply;
    serial_port_.read(&reply, 1);
    
    serial_port_.resumeStream();
    
    if(reply == 'z') return true;
    
    ROS_ERROR("USB2_F_7001 - %s - Did not get reply - %d", __FUNCTION__, (int)reply);
    return false;
}

// TODO: Implement this function!
bool USB2_F_7001::transmitExtendedFrame(std::string * frame)
{
    if(!open_) return false;
    
    return false;
}

// TODO: Implement this function!
int USB2_F_7001::getVersion()
{
    if(open_) return false;
    
    return 0;
}

// TODO: Implement this function!
int USB2_F_7001::getSerialNumber()
{
    if(open_) return false;
    
    return 0;
}

void USB2_F_7001::newFrameCallback(std::string * frame)
{
    receivedFrameCallback(frame);
}

// EOF
