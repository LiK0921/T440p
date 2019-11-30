#ifndef __PWM_H__
#define __PWM_H__

#include "system.h"

void TIM4_Init(void);
void TIM3_Init(void);
void TIM1_Init(void);

void Friction_PWM(int16_t pwm1,int16_t pwm2);
void Servo_PWM(int16_t pwm1);
#define PWM1  TIM4->CCR3
#define PWM2  TIM4->CCR4

#endif

