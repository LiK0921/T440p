#ifndef __JUDGEMENT_H
#define __JUDGEMENT_H

#include "system.h"

enum JudgCmdID{ 
	Competition_Process_Information = 0x0001,						
	Real_time_Blood_Information = 0x0002,
	Real_time_Shoot_Information = 0x0003,
};

typedef enum{ 
	BUFF_TYPE_NONE,						//无效
	FF_TYPE_ARMOR = 0x01,			//防御符
	BUFF_TYPE_SUPPLY = 0X04,	//加血符
	BUFF_TYPE_BULLFTS=0X08,		//加弹符
}eBuffType;

typedef __packed struct
{
	uint8_t flag;
	uint32_t x;				//0无效,1有效
	uint32_t y;
	uint32_t z;
	uint32_t compass;
}tLocData;

typedef __packed struct
{
	uint32_t remainTime;
	uint16_t remainLifeValue;
	float realChassisOutV;
	float realChassisOutA;
	tLocData locData;
	float remainPower;
}tGameInfo;

typedef __packed struct
{
	uint8_t weakId:4;
	uint8_t way:4;
	uint16_t value;
}tRealBloodChangedData;

typedef __packed struct
{
	float realBulletShootSpeed;
	float realBulletShootFreq;
	float realGolfShootSpeed;
	float realGolfShootFreq;
}tRealShootData;

typedef __packed struct
{
float data1;
float data2;
float data3;
}tRobot_Infor;

extern float realPower;
extern tGameInfo Pro_Information;
extern tRealBloodChangedData BloodData;
extern tRealShootData ShootData;

extern u8 Judge_data;
extern char data_num;
extern unsigned char Judge_Rx_Buf[100],Judge_Tx_Buf[100];

void Get_Judgement_data(void);
void Send_Judgement_data(void);
#endif
