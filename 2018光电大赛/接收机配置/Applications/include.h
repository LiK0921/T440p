#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "stm32f4xx.h"
#include "init.h"

#define MAXMOTORS 		(4)		//�������
#define GET_TIME_NUM 	(5)		//���û�ȡʱ�����������
#define CH_NUM 				(8) 	//���ջ�ͨ������

#define ANO_DT_USE_USART2 	//��������2��������
#define GET_TIME_NUM 	(5)		//���û�ȡʱ�����������
#define NVIC_GROUP NVIC_PriorityGroup_3	

//=======================================
/***************�ж����ȼ�******************/
#define NVIC_GROUP NVIC_PriorityGroup_3		//�жϷ���ѡ��
#define NVIC_PWMIN_P			1		//���ջ��ɼ�
#define NVIC_PWMIN_S			1
#define NVIC_TIME_P       2		//��δʹ��
#define NVIC_TIME_S       0
#define NVIC_UART_P				5		//��δʹ��
#define NVIC_UART_S				1
#define NVIC_UART2_P			3		//����2�ж�
#define NVIC_UART2_S			1
/***********************************************/


// CH_filter[],0�����1������2���ţ�3����		
enum
{
 ROL= 0,       //x��
 PIT ,         //y��
 THR ,         //����
 YAW ,         //z��
 AUX1 ,        
 AUX2 ,
 AUX3 ,
 AUX4 ,
};


#endif

