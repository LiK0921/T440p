#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <stdio.h>
#include "stdbool.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "sys.h" 
#include "usart.h"	
#include "string.h"
#include "stdlib.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "myiic.h" 
#include "mpu6050.h"

#include "infrared.h"
#include "crc.h"
#include "pwm.h"
#include "can1.h"
#include "can2.h"
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
#include "uart5.h"
#include "judgement.h"
#include "Laser.h"
#include "jy61.h"
#include "led.h"
#include "control.h"

#define abs(x) ((x)>0? (x):(-(x)))

//总决赛
//#define Infantry_Emerge						//步兵-浮现    	 	福宪
//#define Infantry_OpenFire					//步兵-开火     		小威
#define Infantry_Yixiuge2					//步兵-一休哥二世  林宇
//#define Infantry_Carrot						//步兵-萝卜     		备用

//分区赛
//#define Infantry_Elixir			  		//步兵-仙丹     		小威
//#define Infantry_Yixiuge 					//步兵-一休哥   		林宇
//#define Infantry_Carrot						//步兵-萝卜  	 		福宪
//#define Infantry_Sprint  					//步兵-冲刺    		备用
  
typedef struct 
{
		struct 
		{	
			float P[2][4];
			float I[2][4];
			float Level[2][3];
			float IMax;
		}Gimbal;
		
		struct 
		{	
			float P;
			float I;
			float D;
			float IMax;
		}Speed;
		
		struct
		{
			float P[2];
			float I[2];
			float D[2];
			float IMax[2];
		}RM2006;
		
		float Looptime;
}PID_t;

extern PID_t PID;

extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern float pitch,roll,yaw; 		//欧拉角

uint32_t micros(void);
uint32_t millis(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void Parameter_Init(void);
void Stop(void);
void System_Init(void);
void Loop(void);

#endif


