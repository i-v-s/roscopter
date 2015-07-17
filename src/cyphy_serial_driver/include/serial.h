/*  
 *  Mikrokopter FlightControl Serial Interface
 *  Copyright (C) 2010, Cyphy Lab.
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  https://wiki.qut.edu.au/display/cyphy
 *
 *
 *  Note that the part of this code,(SerialInterface(), ~SerialInterface(),output() is borrowed from CCNY asctec_autopilot serial interface communication 
 *  package. However, the core part of this code, getdata(),ParsingData(), and Decode64() were written by Inkyu.
 *
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

#ifndef MIKO_FLIGHTCONTROL_SERIALINTERFACE_H
#define MIKO_FLIGHTCONTROL_SERIALINTERFACE_H

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <math.h>

#include <ros/ros.h>

//#include "crc16.h"
//#include "telemetry.h"
#include "mkstruct.h"

#include <map>

#define MAX_PITCH_THRESHOLD 360
#define MAX_ROLL_THRESHOLD 360
#define MAX_YAW_THRESHOLD 360
#define MAX_HEIGHT_THRESHOLD 200
#define MAX_ACC_THRESHOLD 400
//#define FC_DEBUG

class MKDevice
{
protected:
    virtual void onReceive(char id, void * data, int size) = 0;
    friend class MKSerialInterface;
};

class MKSerialInterface
{
public:
    MKSerialInterface(std::string port, uint32_t speed);
    virtual ~MKSerialInterface();

    void output(char *output, int len);
    void output(unsigned char *output, int len);
    static int decode64(uint8_t * data, uint8_t * end);
    void ParsingData(void);
    void dumpDebug (void);
    void receive();
    void transmit(MKADDR addr, char cmd, const void * data, size_t size);

    int *scan;
    bool status;
    int pt[800];
    int counter;

    bool Initialized;
    int count;
    std::map<MKADDR, MKDevice *> mDevices;
private:
    uint8_t mFrame[128], * mFramePos;
    int mCRC;
    speed_t bitrate(int);
    inline void flush () { tcflush(mDev, TCIOFLUSH);}
    static uint8_t * addCRC(uint8_t * start, uint8_t * end);

    void drain ();
    void stall (bool);
    int wait (int);

    int mDev;
    std::string mPort;
    uint32_t mSpeed;
    speed_t mBaud;
  };
#endif
