#ifndef __CAN2_H__
#define __CAN2_H__

#include "system.h"

typedef struct
{
		int16_t Begin;
		int16_t Current;
		int32_t Total;
		float Count;
}RM06_Angle_t;

extern int16_t RM06_Motor[2];
extern RM06_Angle_t RM06_Angle;

void CAN2_Init(void);
void CAN2_Send_RM2006_Motor(void);
#endif 
