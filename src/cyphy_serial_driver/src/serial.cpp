/*
 *  Mikrokopter FlightControl Serial Interface
 *  Copyright (C) 2010, Cyphy Lab.
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  https://wiki.qut.edu.au/display/cyphy
 *  Note that the part of this code,(SerialInterface(), ~SerialInterface(),output()) is borrowed from CCNY asctec_autopilot serial interface communication 
 *  package. However, the core part of this code, getdata(),ParsingData(), and Decode64() were written by Inkyu.
 *
 *  Fixed and modified by Mike Charikov, 2013, CROC Inc. 
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "flightcontrol.h"
#include "serial.h"

#include <iostream>

// C++ is a horrible version of C
extern "C" {
  #include <unistd.h>
  #include <fcntl.h>
}

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";


extern unsigned char g_ReceivedBytes;
extern unsigned char *g_pRxData;
extern unsigned char g_RxDataLen;
extern unsigned char g_rxd_buffer[RXD_BUFFER_LEN];
extern unsigned char g_buf_debug[300];
extern struct str_Data3D g_Data3D,g_Data3D_Temp;
extern DebugOut_t g_DebugData,g_DebugData_Temp;


MKSerialInterface::MKSerialInterface (std::string port, uint32_t speed):
    serialport_name_ (port), serialport_speed_ (speed), mFramePos(0)
{
    struct termios tio;
    status = false;
    serialport_baud_ = bitrate (serialport_speed_);
    ROS_INFO ("Initializing serial port...");

    dev_ = open(serialport_name_.c_str (),O_RDWR | O_NOCTTY | O_NDELAY);
    ROS_DEBUG ("dev: %d", dev_);
    ROS_ASSERT_MSG (dev_ != -1, "Failed to open serial port: %s %s", serialport_name_.c_str (), strerror (errno));

    ROS_ASSERT_MSG (tcgetattr (dev_, &tio) == 0, "Unknown Error: %s", strerror (errno));

    cfsetispeed (&tio, serialport_baud_);
    cfsetospeed (&tio, serialport_baud_);

    tio.c_iflag = 0;
    tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
    tio.c_iflag |= IGNBRK;

    tio.c_oflag = 0;
    tio.c_oflag &= ~(OPOST | ONLCR);

    tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
    tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

    tio.c_lflag = 0;
    tio.c_lflag |= NOFLSH;
    tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);

    ROS_ASSERT_MSG (tcsetattr (dev_, TCSADRAIN, &tio) == 0, "Unknown Error: %s", strerror (errno));

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    tcflush (dev_, TCIOFLUSH);

    Initialized=false;

    ROS_ASSERT_MSG (dev_ != 0, "Could not open serial port %s", serialport_name_.c_str ());
    ROS_INFO ("Successfully connected to %s, Baudrate %d\n", serialport_name_.c_str (), serialport_speed_);
}

MKSerialInterface::~MKSerialInterface()
{
    ROS_DEBUG ("Destroying Serial Interface");
    flush ();
    close (dev_);
}

  void MKSerialInterface::flush ()
  {
    tcflush (dev_, TCIOFLUSH);
  }

  void MKSerialInterface::drain ()
  {
    ROS_ASSERT_MSG (tcdrain (dev_) == 0, "Drain Error: %s", strerror (errno));
  }

  int MKSerialInterface::wait (int bytes_requested)
  {
    int bytes_available=0;
    unsigned int i=0;

    while (bytes_available < bytes_requested)
    {
      ioctl(dev_,FIONREAD,&bytes_available);
      usleep(1);
      if (i>650 && bytes_available < bytes_requested)
      {
        ROS_ERROR("Timeout: %d bytes available %d bytes requested",bytes_available,bytes_requested);
        return bytes_available;
      }
      i++;
    }
    return bytes_available;
  }

speed_t MKSerialInterface::bitrate (int Bitrate)
{
    switch (Bitrate)
    {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:                 // invalid bitrate
        return B0;
    }
}

  void MKSerialInterface::output (char *output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    serialport_bytes_tx_ += len;
    i = write (dev_, output, len);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
    ROS_DEBUG ("Write completed");
  }

  void MKSerialInterface::output (unsigned char *output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    serialport_bytes_tx_ += len;
    i = write (dev_, output, len);
    if (i != len)
    {
      ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
  }

void MKSerialInterface::read()
{
    uint8_t data[256];
    ssize_t len = ::read(dev_, data, sizeof(data));
    for(uint8_t * x = data, * e = data + len; x < e; x++) switch(uint8_t c = *x)
    {
    case '#':
        if(mFramePos && mFramePos != mFrame) ROS_WARN("Frame without '\\r'");
        mFramePos = mFrame;
        mCRC = c;
        break;
    default:
        if(mFramePos)
        {
            if(mFramePos >= mFrame + sizeof(mFrame))
            {
                ROS_WARN("Too long frame");
                mFramePos = 0;
                break;
            }
            *(mFramePos++) = c;
            mCRC += c;
        }
        break;
    case '\r':
        if(mFramePos)
        {
            if(mFramePos - mFrame < 4)
            {
                ROS_WARN("Too short frame");
                mFramePos = 0;
                break;
            }
            uint8_t crc1 = mFramePos[-2], crc2 = mFramePos[-1];
            uint crc = (mCRC - crc1 - crc2) & 0xFFF;
            if(crc1 != (crc >> 6) + '=' || crc2 != (crc & 0x3F) + '=')
            {
                ROS_WARN("Wrong crc in frame");
                mFramePos = 0;
                break;
            }
            int size = Decode64(mFrame + 2, mFramePos - 2);
            auto i = mDevices.find(mFrame[0]);
            onReceive(mFrame[0], mFrame[1], mFrame + 2, size);
            mFramePos = 0;
        }
        break;
    }
}

int MKSerialInterface::Decode64(uint8_t * data, uint8_t * end)
{
    uint8_t a,b,c,d;
    uint8_t x,y,z;
    uint8_t * dst = data;
    for(uint8_t * src = data; src < end; )
    {
        a = *(src++) - '=';
        b = *(src++) - '=';
        if(src < end)
        {
            c = *(src++) - '=';
            d = *(src++) - '=';
        }
        else
        {
            c = 0;
            d = 0;
        }

        x = (a << 2) | (b >> 4);
        y = ((b & 0x0f) << 4) | (c >> 2);
        z = ((c & 0x03) << 6) | d;

        *(dst++) = x;
        *(dst++) = y;
        *(dst++) = z;
    }
    return dst - data;
}


/*void MKSerialInterface::onReceive(char addr, char id, void * data, int size)
{
    ROS_INFO("Received addr:%c id:%c", addr, id);
    if(addr != 'b') return;
    switch(id)
    {
    case 'D':
        {
            DebugOut_t & o = *(DebugOut_t *)data;
            //for( = (DebugOut_t *)data, * e = (DebugOut_t *)((uint8_t *)data + size); o < e; o++)
            for(int x = 0; x < 32; x++)
                ROS_INFO("param %d = %d", x, o.Analog[x]);

        }

        break;

    }
}*/


 




