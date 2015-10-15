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
#include <stdexcept>
#include <termios.h>
#include <string>
#include <vector>

#include <boost/function.hpp>

#include <cereal_port/CerealPort.h>

#define CANUSB_ERROR   7       // <BELL>
#define CANUSB_OK      13      // <CR>

//! Possible values for the USB-F-7001 bit rate
typedef enum CANUSB_BitRate_
{
    BIT_RATE_10Kbps = 0,
    BIT_RATE_20Kbps = 1,
    BIT_RATE_50Kbps = 2,
    BIT_RATE_100Kbps = 3,
    BIT_RATE_125Kbps = 4,
    BIT_RATE_250Kbps = 5,
    BIT_RATE_500Kbps = 6,
    BIT_RATE_800Kbps = 7,
    BIT_RATE_1000Kbps = 8
    
} CANUSB_BitRate;

/*! \class CANUSB CANUSB.h "inc/CANUSB.h"
*  \brief C++ class the CAN USB adapter CANUSB.
*
*/
class CANUSB
{
    
public:
    //! Constructor
    CANUSB(std::string * serial_port_name, boost::function<void(std::string*)> f);
    //! Destructor
    ~CANUSB();

    //! Open the CAN bus
    /*! 
     * This opens the CAN bus with the desired baud rate.
     * 
     * \param bit_rate   Bit rate of the serial port.
     *
     */
    bool openCANBus(CANUSB_BitRate bit_rate);
    
    //! Close the CAN bus
    /*!
     * This call closes the CAN bus.
     *
     * \param bit_rate   Bit rate of the serial port.
     *
     */
    bool closeCANBus();
    
    //! Check whether the CAN bus is open or not.
    bool CANBusOpen() { return open_; }
    
    //! Reset
    /*! 
     * Resets the CANUSB.
     *
     *  \return True is succesful, false if otherwise.
     */
    bool reset();
    
    //! Set the bit rate
    /*! 
     * Sets the USB2-F-7001 bit rate.
     * Values are   10Kbp2
     *              20Kbps
     *              50Kbps
     *              100Kbps
     *              125Kbps
     *              250Kbps
     *              500Kbps
     *              800Kbps
     *              1000Kbps
     *
     *  \return True is succesful, false if otherwise.
     */
    bool setCANBitRate(CANUSB_BitRate bit_rate);
    //! Get the current bit rate
    CANUSB_BitRate getCANBitRate() { return bit_rate_; }
    
    //! Transmit standard frame
    /*!
     *  This function allows to send a standard CAN frame.
     *
     *  \param frame    Standard data frame to send.
     * 
     *  \return True is succesful, false if otherwise.
     */
    bool transmitStandardFrame(std::string * frame);
    
    //! Transmit extended frame
    /*!
     *  This function allows to send an extended CAN frame.
     *
     *  \param frame    Extended data frame to send.
     * 
     *  \return True is succesful, false if otherwise.
     */
    bool transmitExtendedFrame(std::string * frame);
    
    //! Get the version
    int getVersion();
    //! Get the serial number
    int getSerialNumber();
    
    
private:
    //! New CAN frame callback.
    /*!
     *  This function allows to receive CAn frames.
     *
     *  \param frame    Data frame to receive.
     * 
     */
    void newFrameCallback(std::string * frame);
    
    //! Bit rate
    CANUSB_BitRate bit_rate_;
    
    //! CAN bus state
    bool open_;
    
    //! Serial port
    cereal::CerealPort serial_port_;
		
    //! Callback function
    boost::function<void(std::string*)> receivedFrameCallback;
};

// EOF
