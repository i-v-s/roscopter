/*
 *  Mikrokopter Flight control Serial Interface
 *  Copyright (C) 2010, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
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

#ifndef MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H
#define MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <stdarg.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>


#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sys/time.h>
#include "mkstruct.h"
#include "serial.h"
#include "sensor_msgs/Imu.h"  //For height
#include "cyphy_serial_driver/mikoImu.h"
#include "cyphy_serial_driver/mikoCmd.h"
#include <control_toolbox/pid.h> //For PID height control


#define TRUE 1
#define FALSE 0
#define YES 1
#define NO 0

#define FC_ADDRESS 2



class FlightControl: public MKDevice
{
private:

      ros::Timer timer_;
    
      double freq_;
      std::string port_;
      int speed_;
      bool Throttle_Direction;


    ros::Publisher pub;         //mikoImu pub
    ros::Publisher pub_stdImu;  //sensors_msgs::Imu pub

    sensor_msgs::Imu stdImuMsg;
    tf::Quaternion tfImuQuaternion;
    geometry_msgs::Quaternion geometryImuQuaternion;

    //ros::Publisher pub_pose2D;

    
    uint64_t time;
    
    DesiredPosition_t DesiredPosition;
    Attitude_t MyAttitude;
    void publishData(const DebugOut_t &data);

protected:
    virtual void onReceive(char id, void * data, int size);

public:
      MKSerialInterface* serialInterface_;

      FILE *fd,*fd_h,*fd_debug;
      cyphy_serial_driver::mikoImu mikoImu;
      ros::Time last_time;
      ros::Time last_time_;
      ros::Time current_time;
      ros::Time current_time_;

      ExternControl_t	ExternControl;
      ros::Subscriber mikoCmdSubscriber;
      int myYawAngle;

      FlightControl ();
      virtual ~FlightControl();
      void AddCRC(uint16_t datelen);
      void SendOutData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...);
      void enablePolling (uint16_t request, uint16_t interval);
      void spin (const ros::TimerEvent & e);
      void mikoCmdCallback (const cyphy_serial_driver::mikoCmd& msg);
      
      unsigned long long time_helper(void);
}; // end class FlightControl

#endif
