#ifndef __JY_61_H__
#define __JY_61_H__

#include "system.h"

struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};

struct SDStatus
{
	short sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};

//extern struct STime stcTime;
//extern struct SAcc	stcAcc;
//extern struct SGyro stcGyro;
//extern struct SAngle stcAngle;
//extern struct SMag stcMag;
//extern struct SDStatus stcDStatus;
//extern struct SPress stcPress;
//extern struct SLonLat stcLonLat;
//extern struct SGPSV stcGPSV;
//extern int trans_success_flag;
//extern float accx ,accy ,accz;
//extern float gyrox ,gyroy ,gyroz;
//extern float roll ,pitch ,yaw;
extern float Chassis_pitch;
extern int16_t sensor_num;

//void Data_Put_Char(unsigned char DataToSend);
//void Data_Put_String(unsigned char *Str);
//void USART4_Init(void);
//void USART4_SendChar(unsigned char b);
//void put_sensor_data(void);	
void sensor_Data_handle(unsigned char sensor_data);
#endif
