#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"
#include "include.h"
#include "Ano_FcData.h"


extern float Roll,Pitch,Yaw;
extern float mpu6050_tmp[ITEMS];

void MPU6050_Get(u8 data);

#endif

