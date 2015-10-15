/***
 * Motor Controller
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sh2interface/encoder.h"
#include "sh2interface/power_status.h"
#include "sh2interface/sh2_tx.h"
#include "serial/serial.h"
#include <string>
#include <iostream>
#include <cstdio>

#define TX_DATA_SIZE 9  // size of send data
#define RX_DATA_SIZE 16 // size of receive data
#define CONTROL_F 100 // rate of sending control input [Hz]

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
int PGain = 30;
int IGain = 5;
int DGain = 1;



uint8_t tx_data[TX_DATA_SIZE];

/*
** Callback to make message to send to SH2
*/
void chatterCallback(const sh2interface::sh2_tx::ConstPtr& msg)
{
  tx_data[2] = msg->command;
  tx_data[3] = msg->id;
  tx_data[4] = msg->data[0];
  tx_data[5] = msg->data[1];
  tx_data[6] = msg->data[2];
  tx_data[7] = msg->data[3];
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  
//  // Debug
//  for(int i = 0; i < 9; i++)
//  {
//    cout << (int)tx_data[i] << " ";
//  }
//  cout << endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sh2interface_node");
  ros::NodeHandle n;
  
  // Publisher and Subscriber
  ros::Publisher enc_pub = n.advertise<sh2interface::encoder>("encoder", 10);
  ros::Publisher ps_pub = n.advertise<sh2interface::power_status>("power_status", 10);
  ros::Subscriber sub = n.subscribe<sh2interface::sh2_tx>("sh2_tx", 10, chatterCallback);

	if(ros::param::has("/sh2interface_node/SetPGain"))
	{
		ros::param::get("/sh2interface_node/SetPGain", PGain);
	}
	if(ros::param::has("/sh2interface_node/SetIGain"))
	{
		ros::param::get("/sh2interface_node/SetIGain", IGain);
	}
	if(ros::param::has("/sh2interface_node/SetDGain"))
	{
		ros::param::get("/sh2interface_node/SetDGain", DGain);
	}
  
  // TODO: parameterize
  // serial port
  //string port("/dev/ttyUSB0");
  string port("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A8006lMt-if00-port0");
  // baudrate
  unsigned long baud = 57600;
  // port, baudrate, no timeout(get the received data at the time)
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(0));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
  
  // Initialize Buffers
  // for Transmit
  memset(tx_data, 0, TX_DATA_SIZE);
  tx_data[0] = (char)0xff; // header
  tx_data[1] = (char)0xff; // header
  // for Receive
  uint8_t rx_data[RX_DATA_SIZE*2];
  memset(rx_data, 0, RX_DATA_SIZE*2);
  int end = 0;  // this means the number of buffered data

  // Initialize SH2
  // Motor disable
  tx_data[2] = 0x31; // command
  tx_data[3] = 0x01; // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  tx_data[3] = 0x11;  // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  ros::Duration(0.2).sleep();
  // Motor enable
  tx_data[2] = 0x30; // command
  tx_data[3] = 0x01; // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  tx_data[3] = 0x11;  // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  ros::Duration(0.2).sleep();
  // Start to send message
  tx_data[2] = 0x21; // command
  tx_data[3] = 0x01; // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  tx_data[3] = 0x11;  // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  ros::Duration(0.2).sleep();
  
  //SetGain
  tx_data[2] = 0x61;	// command
  tx_data[3] = 0x01;	// id
  tx_data[4] = (char)PGain;//0;//30;		// kp
  tx_data[5] = (char)IGain;//0;//5;		// ki
  tx_data[6] = (char)DGain;//0;//1;		// kd
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  tx_data[2] = 0x62;	// command
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);


  ros::Rate loop_rate(CONTROL_F);
  
  sh2interface::encoder enc;
  sh2interface::power_status ps;
  
  cout << "Start" << endl;
  // Main Loop
  while(ros::ok())
  {
    // Send message to SH2
    my_serial.write(tx_data, TX_DATA_SIZE);
    
    // Receive message from SH2
    uint8_t data[RX_DATA_SIZE];
    int result = my_serial.read(data, RX_DATA_SIZE);  // return value: the number of received data
    
    // Buffering
    int end_pre = end;
    for(int i = 0; i < result; i++)
    {
      if((end_pre + i) > RX_DATA_SIZE*2) break; // overflow
      rx_data[end_pre + i] = (int8_t)data[i]; // push_back
      end++;
    }
    
    // Proccess buffered data
    if((rx_data[0] == 0xff) && (rx_data[1] == 0xff)) // if header data is already top of buffer
    {
      if(end >= RX_DATA_SIZE) // if the num of buffered data >= defined value
      {
//        // Debug
//        for(int i = 0; i < RX_DATA_SIZE; i++)
//        {
//          cout << i << " : " << (int)rx_data[i] << endl;
//        }
        
        
        // convert data
        // Status
        enc.left_status = rx_data[2];
        enc.right_status = rx_data[8];
        
        // Left Flipper Encoder
        enc.e_data[0] = (short)((rx_data[10] << 8) + (short)rx_data[11]);
        // Right Flipper Encoder
        enc.e_data[1] = (short)((rx_data[12] << 8) + (short)rx_data[13]);
        // Left Crawler Encoder
        enc.e_data[2] = (short)(rx_data[4] << 8) + (short)rx_data[5];
        // Right Crawler Encoder
        enc.e_data[3] = (short)(rx_data[6] << 8) + (short)rx_data[7];
        
        // PC Buttery level
        ps.pc_battery_status = rx_data[14];
        // Emergency switch
        ps.emergency_sw_status = rx_data[15];
        
        // Publish message
        enc_pub.publish(enc);
        ps_pub.publish(ps);
        
        // shift bufferd data
        for(int i = 0; i < (end - RX_DATA_SIZE); i++)
        {
          rx_data[i] = rx_data[end + i];
        }
        end = end - RX_DATA_SIZE;
      }
    }
    else
    {
      if(end >= 2)
      {
        while((rx_data[0] != 0xff) || (rx_data[1] != 0xff)) // top of buffered data is not header data
        {
          for(int i = 1; i < end; i++)
          {
            rx_data[i-1] = rx_data[i];
          }
          end = end - 1;
          if(end < 2) break;
        }
      }
    }
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }
  
  // Stop motors
  tx_data[2] = 0x31; // command
  tx_data[3] = 0x01; // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  tx_data[3] = 0x11;  // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  ros::Duration(1.0).sleep();

  // Stop to send message
  tx_data[2] = 0x20; // command
  tx_data[3] = 0x01; // id
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  ros::Duration(1.0).sleep();

  // turn off led
  tx_data[2] = 0xb0;	// command
  tx_data[3] = 0x21;		// id
  tx_data[4] = 0x00;	// red
  tx_data[5] = 0x00;	// green
  tx_data[6] = 0x00;	// blue
  tx_data[7] = 0;		// interval
  tx_data[8] = 0;
  for(int i = 0; i < 8; i++)
  {
    tx_data[8] += tx_data[i]; // CheckSum
  }
  my_serial.write(tx_data, TX_DATA_SIZE);
  
  cout << "sh2interace_node End" << endl;

  return 0;
}
