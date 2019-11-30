#include "stm32f4xx.h"
#include "init.h"
#include "delay.h"
#include "led.h"
#include "timer.h"
#include "key.h"
#include "usart.h"
#include "include.h"
#include "Parameter.h"
#include "pwm_in.h"
#include "can1.h"
#include "drv_w25qxx.h"
#include "AnoParameter.h"
#include "Ano_FcData.h"
#include "data_transfer.h"
#include "parameter.h"

u8 task_state;
	
u8 All_Init()
{
	delay_init(168);		//初始化延时函数
	
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 	//滴答时钟
	
	Flash_Init();             		//板载FLASH芯片驱动初始化
	
	Para_Data_Init();              		//参数数据初始化
	
	PWM_IN_Init();
	
	Delay_ms(400);
	
	Usart2_Init(500000);
	
	Usart3_Init(115200);
	
	CAN1_Configuration();   
	
	Delay_ms(100);						//延时
	
	All_PID_Init();               		//PID初始化
	                                    
	ANO_DT_SendString("SYS init OK!",sizeof("SYS init OK!"));
	
	task_state = 1;
	
	Cycle_Time_Init();
	
	return (1);
}


