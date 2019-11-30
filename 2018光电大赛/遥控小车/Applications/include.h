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


//================ϵͳ===================
#define HW_TYPE	05
#define HW_VER	1
#define SOFT_VER 32
#define BL_VER	0
#define PT_VER	400


#define ANO_DT_USE_USART2 				//��������2��������
#define ANO_DT_USE_USB_HID				//�����ɿ�USBHID������λ������
//=======================================

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

//CANͨ����ʼ������
#define RATE_BUF_SIZE 10
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                    //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								    //�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	    //buf��for filter
	int32_t round_cnt;										//Ȧ��
	float filter_rate;									//�ٶ�
	float   rate;
	float ecd_angle;											//�Ƕ�
}Encoder;



#endif

