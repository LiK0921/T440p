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

	
u8 All_Init()
{
	delay_init(168);		//初始化延时函数
	
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 	//滴答时钟
	
	PWM_IN_Init();
	
	Key_Init();
	
	LED_Init();
	
	Usart2_Init(500000);
	
	Para_Init();							//参数初始化
	
	Delay_ms(100);						//延时

	LED0=0;				  	//先点亮红灯
	
	Cycle_Time_Init();
	
	return (1);
}


