/*
 *  Mikrokopter Flight control Serial Interface
 *  Based on CYPHY lab realization
 *  Copyright (C) 2010, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *  Fixed and modified by Mike Charikov, 2013 CROC Inc.
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

#include "serial.h"
#include "flightcontrol.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flightcontrol");
    ros::NodeHandle nh;

    double freq;
    std::string port;
    int speed;
    if(!nh.getParam("/MikoControl/device", port)) port = "/dev/ttyUSB0";
    if(!nh.getParam("/MikoControl/baud", speed)) speed = 57600;
    if(!nh.getParam("/MikoControl/frequency", freq)) freq = 50.0;
    if (freq <= 0.0) ROS_FATAL ("Invalid frequency param");

    MKSerialInterface serial(port, speed);
    FlightControl flightcontrol(nh, &serial);

    ros::Rate rate(freq);

    while(ros::ok())
    {
        serial.receive();
        flightcontrol.spin();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

FlightControl::FlightControl(ros::NodeHandle &nh, MKSerialInterface * serial): mSerial(serial)
{
    serial->mDevices.insert(std::make_pair(FC_ADDRESS, this));

	ROS_INFO ("Creating FlightControl Interface");
	
    mPub = nh.advertise<cyphy_serial_driver::mikoImu>("mikoImu", 100);
    mPub_stdImu = nh.advertise<sensor_msgs::Imu>("imu/data", 100);

	mikoCmdSubscriber = nh.subscribe("c_command", 100, &FlightControl::mikoCmdCallback, this);

	// Reference input initialization
	ExternControl.Digital[0] =0;
	ExternControl.Digital[1] =0;
	ExternControl.RemoteButtons =0;
	ExternControl.Pitch =0;
	ExternControl.Roll =0;
	ExternControl.Yaw =0;
	ExternControl.Throttle =0;
	ExternControl.Height =0;
	ExternControl.free =0;
	ExternControl.Frame =0;
	ExternControl.Config =1;
	Throttle_Direction=true;

    memset(&mDebugData, 0, sizeof(mDebugData));
}
  

FlightControl::~FlightControl()
{
    ROS_INFO ("Destroying FlightControl Interface");
}

void FlightControl::publishData(const DebugOut_t &data)
{
    //=================================================================
    // Coordinate and unit conversion from Mikrokopter to Cyphy model.
    //=================================================================

    // Put negative(minus) all angles to covert all axies into a right hand coordinate.
    // Please have a look MK Coordinate system V1.0 documentation in order to understand coordinate systems.

    MyAttitude.AnglePitch = -DEG_TO_RAD(data.Analog[ANGLE_PITCH]/10.);
    MyAttitude.AngleRoll = -DEG_TO_RAD(data.Analog[ANGLE_ROLL]/10.);

    // To make IMU Yaw coordinate same as Laser Yaw coordinate.
    if(data.Analog[ANGLE_YAW]>=180)
        MyAttitude.AngleYaw = (double)(data.Analog[ANGLE_YAW]-360.);
    else
        MyAttitude.AngleYaw = (double)data.Analog[ANGLE_YAW];

    MyAttitude.AngleYaw = DEG_TO_RAD(MyAttitude.AngleYaw);

    // Normalize Acceleration data as 1G.
    // Please have a look MK Coordinate system V1.0 documentation in order to understand coordinate systems.
    MyAttitude.ACCX = -(data.Analog[ACC_X]/606.)*C_g;
    MyAttitude.ACCY = (data.Analog[ACC_Y]/603.)*C_g;
    //MyAttitude.ACCZ = (g_DebugData.Analog[ACC_Z_FILTER]/15.5); // m/s^2
    MyAttitude.ACCZ = (data.Analog[ACC_Z_ADC]/15.5); // m/s^2

    //
    //	The unit of angle is radian and m/s^2 for acceleration (not g)
    //
    mikoImu.anglePitch = MyAttitude.AnglePitch;
    mikoImu.angleRoll = MyAttitude.AngleRoll;
    mikoImu.angleYaw = MyAttitude.AngleYaw;
    mikoImu.linear_acceleration.x = MyAttitude.ACCX;
    mikoImu.linear_acceleration.y = MyAttitude.ACCY;
    mikoImu.linear_acceleration.z = MyAttitude.ACCZ;
    mikoImu.stick_throttle = data.Analog[GAS];
    mikoImu.barome_height = data.Analog[HEIGHT];

    stdImuMsg.linear_acceleration.x=MyAttitude.ACCX;
    stdImuMsg.linear_acceleration.y=MyAttitude.ACCY;
    stdImuMsg.linear_acceleration.z=MyAttitude.ACCZ;

    tfImuQuaternion = tf::createQuaternionFromRPY(MyAttitude.AngleRoll, MyAttitude.AnglePitch, MyAttitude.AngleYaw);
    tf::quaternionTFToMsg(tfImuQuaternion, geometryImuQuaternion);

    stdImuMsg.orientation = geometryImuQuaternion;
    stdImuMsg.angular_velocity_covariance[0]=-1;
    stdImuMsg.linear_acceleration_covariance[0]=-1;

    if(data.Analog[BATT] < BATT_MAX) mikoImu.batt = data.Analog[BATT];


    for (int i=0; i<32; i++) mikoImu.debugData[i] = data.Analog[i];

    mPub.publish(mikoImu);
    mPub_stdImu.publish(stdImuMsg);
}

void FlightControl::onReceive(char id, void * data, int size)
{
    switch(id)
    {
    case 'C'://67: //Set 3D-Data Interval
        if(size < sizeof(mData3D))
            ROS_WARN("3D-Data packet with wrong size");
        else
            memcpy(&mData3D, data, sizeof(mData3D));
        break;
    case 'D'://68: // Debug Request
        if(size < sizeof(DebugOut_t))
        {
            ROS_WARN("Debug packet with wrong size");
            return;
        }
        else
        {
            DebugOut_t &dd = *(DebugOut_t *) data;

            if(
                abs(dd.Analog[ANGLE_PITCH]) > 1000 ||
                abs(dd.Analog[ANGLE_ROLL])  > 1000 ||
                abs(dd.Analog[ANGLE_YAW])   > 1000 ||
                abs(dd.Analog[ACC_X])       >  700 ||
                abs(dd.Analog[ACC_Y])       >  700 ||
                abs(dd.Analog[ACC_Z_ADC])   >  700
            )
            {
                ROS_INFO("Wrong packet received: %d %d %d %d %d %d ",
                         dd.Analog[ANGLE_PITCH],
                         dd.Analog[ANGLE_ROLL],
                         dd.Analog[ANGLE_YAW],
                         dd.Analog[ACC_X],
                         dd.Analog[ACC_Y],
                         dd.Analog[ACC_Z_ADC]);
            }
            else
            {
                mDebugData = dd;
                ROS_INFO("Good packet received: %d %d %d %d %d %d ",
                         dd.Analog[ANGLE_PITCH],
                         dd.Analog[ANGLE_ROLL],
                         dd.Analog[ANGLE_YAW],
                         dd.Analog[ACC_X],
                         dd.Analog[ACC_Y],
                         dd.Analog[ACC_Z_ADC]);
                break;
            }
        }
    }
}


void FlightControl::spin()
{
	int nLength=0;
	uint8_t interval=5;
	
    mSerial->transmit(FC_ADDRESS, 'd', &interval, sizeof(interval)); // Request debug data from FC
    mikoImu.header.stamp = ros::Time::now();
}


void FlightControl::mikoCmdCallback(const cyphy_serial_driver::mikoCmd& msg)
{
    ExternControl.Pitch = msg.pitch;
    ExternControl.Roll = msg.roll;
    ExternControl.Yaw = msg.yaw;
    ExternControl.Throttle = msg.throttle;

    if(ExternControl.Pitch >=80) ExternControl.Pitch=80;
    if(ExternControl.Pitch <=-80) ExternControl.Pitch=-80;

    if(ExternControl.Roll >=80) ExternControl.Roll=80;
    if(ExternControl.Roll <=-80) ExternControl.Roll=-80;

    if(ExternControl.Yaw >=60) ExternControl.Yaw=60;
    if(ExternControl.Yaw <=-60) ExternControl.Yaw=-60;

    if(ExternControl.Throttle <= 0) ExternControl.Throttle =0;


    //ROS_INFO (">>>> %d, %d, %d, %d",ExternControl.Pitch, ExternControl.Roll, ExternControl.Yaw, ExternControl.Throttle);

    mSerial->transmit(FC_ADDRESS, 'b', &ExternControl, sizeof(ExternControl));
}
