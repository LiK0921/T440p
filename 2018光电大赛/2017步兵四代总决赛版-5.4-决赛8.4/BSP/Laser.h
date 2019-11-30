#ifndef __LASER_H
#define __LASER_H

#include "system.h"

#define Laser_On GPIO_SetBits(GPIOC,GPIO_Pin_15)
#define Laser_Off GPIO_ResetBits(GPIOC,GPIO_Pin_15)

void Laser_Init(void);
#endif

