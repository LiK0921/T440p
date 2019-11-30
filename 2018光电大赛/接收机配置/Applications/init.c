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
	delay_init(168);		//��ʼ����ʱ����
	
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//�ж����ȼ��������
	
	SysTick_Configuration(); 	//�δ�ʱ��
	
	PWM_IN_Init();
	
	Key_Init();
	
	LED_Init();
	
	Usart2_Init(500000);
	
	Para_Init();							//������ʼ��
	
	Delay_ms(100);						//��ʱ

	LED0=0;				  	//�ȵ������
	
	Cycle_Time_Init();
	
	return (1);
}


