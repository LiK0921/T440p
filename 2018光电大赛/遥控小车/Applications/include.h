#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "stm32f4xx.h"
#include "init.h"
#include <stdarg.h>
#include "LostCounter.h"
#include "protocal.h"
#include "superviseTask.h"
#include "parameter.h"
#include "pid.h"


//================系统===================
#define HW_TYPE	05
#define HW_VER	1
#define SOFT_VER 32
#define BL_VER	0
#define PT_VER	400


#define ANO_DT_USE_USART2 				//开启串口2数传功能
#define ANO_DT_USE_USB_HID				//开启飞控USBHID连接上位机功能
//=======================================

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

//CAN通道初始化参数
#define RATE_BUF_SIZE 10
typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                    //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								    //滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	    //buf，for filter
	int32_t round_cnt;										//圈数
	float filter_rate;									//速度
	float   rate;
	float ecd_angle;											//角度
}Encoder;



#endif

