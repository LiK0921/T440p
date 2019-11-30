#ifndef __INFRARED_H__
#define __INFRARED_H__

#include "system.h"

void Infrared_GPIO_Init(void);
void Infrared_Init(void);
void Infrared_On(void);
void Infrared_Off(void);
void TIM2_Init(void);
void TIM2_Start(void);
void TIM5_Init(void);
void TIM5_Start(void);
	
#endif
