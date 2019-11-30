#include "stm32f4xx.h"
#include "init.h"
#include "delay.h"
#include "led.h"
#include "timer.h"
#include "key.h"
#include "usart.h"


	
u8 All_Init()
{
	delay_init(168);		//初始化延时函数
	
	//SysTick_Configuration(); 	//滴答时钟
	
	Usart2_Init(115200);
	
	Key_Init();
	
	LED_Init();
	LED0=0;				  	//先点亮红灯
	
	Cycle_Time_Init();
	
	return (1);
}


