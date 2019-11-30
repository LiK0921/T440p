#ifndef __CAN1_H
#define __CAN1_H

#include "system.h"

extern int16_t Motor_chassis[4][2];

typedef struct
{
		int16_t Power;
		bool Update_flag;
		int Num;
}Measure_t;
		
typedef	struct
{
		int16_t Joule;
		int16_t outV;
		int16_t outA;
		float 	Power;
}Judge_t;


void CAN1_Init(void);
void CAN1_Send_Chassis_Motor(void);
void CAN1_Send_Gimbal_Motor(void);
#endif

