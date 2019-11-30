#include "pwm.h"

void TIM4_Init(void)	//TIM4  Ħ����
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_TIM4);    
	
	tim.TIM_Prescaler = 84-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
	tim.TIM_Period = 2499;   //25ms	��������
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ��Ϊ1�Ļ����2
	TIM_TimeBaseInit(TIM4,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		//ѡ��ʱ��ģʽ
	oc.TIM_OutputState = TIM_OutputState_Enable;		//ѡ������Ƚ�״̬
	oc.TIM_OutputNState = TIM_OutputState_Disable;	//ѡ�񻥲�����Ƚ�״̬
	oc.TIM_Pulse = 0;		//���ô�װ�벶��Ƚ���������ֵ
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		//�����������
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//���û����������
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		//ѡ�����״̬�µķǹ���״̬
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		//ѡ�񻥲�����״̬�µķǹ���״̬
	TIM_OC3Init(TIM4,&oc);		//ͨ��3
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM4,&oc);		//ͨ��4
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	TIM_Cmd(TIM4,ENABLE);
	PWM1 = 1000;		
	PWM2 = 1000;
}

void Friction_PWM(int16_t pwm1,int16_t pwm2)		//Ħ���ֵĽ�������Ϊ�ߵ�ƽ����Լ630us ��ʼת������Ϊ�ߵ�ƽ����Լ1ms
{
	PWM1 = pwm1+1000;	
	PWM2 = pwm2+1000;
}

int16_t ServoInit;
void TIM3_Init(void)	//TIM3
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1, GPIO_AF_TIM3);      
	
	tim.TIM_Prescaler = 1680-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
	tim.TIM_Period = 999;   //20ms	��������
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ��Ϊ1�Ļ����2
	TIM_TimeBaseInit(TIM3,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		//ѡ��ʱ��ģʽ
	oc.TIM_OutputState = TIM_OutputState_Enable;		//ѡ������Ƚ�״̬
	oc.TIM_OutputNState = TIM_OutputState_Disable;	//ѡ�񻥲�����Ƚ�״̬
	oc.TIM_Pulse = 0;		//���ô�װ�벶��Ƚ���������ֵ
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		//�����������
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//���û����������
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		//ѡ�����״̬�µķǹ���״̬
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		//ѡ�񻥲�����״̬�µķǹ���״̬
	
//	TIM_OC3Init(TIM3,&oc);		//ͨ��3
//	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3,&oc);		//ͨ��4
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	
	TIM_Cmd(TIM3,ENABLE);
	TIM3->CCR4 = 0;
}

//void Servo_PWM(int16_t pwm)
//{
//	pwm = abs(pwm);
//	TIM3->CCR4 = pwm;
//}

void TIM1_Init(void)	//TIM1
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		

	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_TIM1);     
	
	tim.TIM_Prescaler = 3360-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
	tim.TIM_Period = 999;   //20ms	��������
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ��Ϊ1�Ļ����2
	TIM_TimeBaseInit(TIM1,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		//ѡ��ʱ��ģʽ
	oc.TIM_OutputState = TIM_OutputState_Enable;		//ѡ������Ƚ�״̬
	oc.TIM_OutputNState = TIM_OutputState_Disable;	//ѡ�񻥲�����Ƚ�״̬
	oc.TIM_Pulse = 0;		//���ô�װ�벶��Ƚ���������ֵ
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		//�����������
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//���û����������
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		//ѡ�����״̬�µķǹ���״̬
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		//ѡ�񻥲�����״̬�µķǹ���״̬
	
	TIM_OC2Init(TIM1,&oc);		//ͨ��4
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
	TIM1->CCR2 = 0;
}

void Servo_PWM(int16_t pwm)
{
	pwm = abs(pwm);
	TIM1->CCR2 = pwm;
}

