#ifndef MKSTRUCT_H
#define MKSTRUCT_H

// MAX when hovering
//#define MAX_PITCH 10
//#define MAX_ROLL 10
//#define MAX_YAW 100

#define YAW_SCALE_FACTOR 10.
#define ACC_X_K 35
#define ACC_Y_K 35
#define BATT_MAX 200


#define ANGLE_PITCH 0
#define ANGLE_ROLL 1

#define ACC_X 2
#define ACC_Y 3
#define GYRO_YAW 4

#define HEIGHT 5

#define ACC_Z_ADC 6

#define GAS 7 // #define GAS 9 //

#define COMPASS 8

#define BATT 9 // #define BATT 8

#define RECIEVER 10

#define ANGLE_YAW 11 // GYRO_COMPASS

#define MOTOR_FRONT 12
#define MOTOR_REAR 13
#define MOTOR_LEFT 14
#define MOTOR_RIGHT 15

#define ACC_Z_FILTER 20 // not true
#define STICK_YAW 23 // not true
#define STICK_PITCH 24 // not true
#define STICK_ROLL 25 // not true

#define GPS_Nick 30
#define GPS_Roll 31



#define HIGH_VALUE 5 // HohenWert air pressure sensor value


#define C_PI 3.141592653589793238462643383279
#define DEG_TO_RAD(x) (x*0.0174532925f)
#define RAD_TO_DEG(x) (x*57.2957795f)
#define BOUNDARY 50   // in mm
#define C_g 9.81   //9.8 m/s^2
#define C_K 0.5   // Complementary filter gain
#define Kp_Pitch 12.
#define Kp_Roll 12.

#define Kp_vel_X 3.
#define Kp_vel_Y 12.
#define Kp_pos_X 10.
#define Kp_pos_Y 10.

#define GAS_PARAM 3.32

typedef struct
{
    double x;
    double y;
    double z;
    double theta;
} __attribute__((packed)) Position_t;

typedef struct
{
    int pitch;
    int roll;
    int yaw;
} Control_t;

typedef struct
{
    double x;
    double y;
    double z;
} Velocity_t;


struct str_Data3D
{
    signed int  angle[3]; // pitch, roll, yaw in 0,1°
    signed char Centroid[3];
    signed char reserve[5];
};

typedef struct
{
    uint8_t Digital[2];
    int16_t Analog[32];
} __attribute__((packed)) DebugOut_t;

// Although I tried to change data type of Pitch and Roll, Yaw as int16_t, it only generated enormous delay
// of request data(Angle pitch, AccX...). I have no idea what is the problem. Therefore, roll back to the
// original data structure of Mikrokopter.
#if 1
  typedef struct
  {
    uint8_t	Digital[2];
    uint8_t	RemoteButtons;
    int8_t	Pitch;
    int8_t	Roll;
    int8_t	Yaw;
    uint8_t  Throttle;
    int8_t	Height;
    uint8_t	free;
    uint8_t	Frame;
    uint8_t	Config;
    } ExternControl_t;
#endif


typedef struct
{
  double AnglePitch;
  double AngleRoll;
  double AngleYaw;
  double ACCX;
  double ACCY;
  double ACCZ;
} __attribute__((packed)) Attitude_t;




typedef struct
{
  double x;
  double y;
  double z;
  double yaw;
}__attribute__((packed)) DesiredPosition_t;

enum MKADDR
{
    FC_ADDRESS      = 'b',
    NC_ADDRESS      = 'c',
    MK3MAG_ADDRESS  = 'd',
    BL_CTRL_ADDRESS = 'f'
};

#endif // MKSTRUCT_H
