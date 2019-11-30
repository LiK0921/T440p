#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "stm32f4xx.h"
#include "init.h"

#define MAXMOTORS 		(4)		//电机数量
#define GET_TIME_NUM 	(5)		//设置获取时间的数组数量
#define CH_NUM 				(8) 	//接收机通道数量

#define ANO_DT_USE_USART2 	//开启串口2数传功能
#define GET_TIME_NUM 	(5)		//设置获取时间的数组数量
#define NVIC_GROUP NVIC_PriorityGroup_3	

//=======================================
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3		//中断分组选择
#define NVIC_PWMIN_P			1		//接收机采集
#define NVIC_PWMIN_S			1
#define NVIC_TIME_P       2		//暂未使用
#define NVIC_TIME_S       0
#define NVIC_UART_P				5		//暂未使用
#define NVIC_UART_S				1
#define NVIC_UART2_P			3		//串口2中断
#define NVIC_UART2_S			1
/***********************************************/


// CH_filter[],0横滚，1俯仰，2油门，3航向		
enum
{
 ROL= 0,       //x轴
 PIT ,         //y轴
 THR ,         //油门
 YAW ,         //z轴
 AUX1 ,        
 AUX2 ,
 AUX3 ,
 AUX4 ,
};


#endif

