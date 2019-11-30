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

//================������===================
#define ACC_ADJ_EN 									//�Ƿ�����У׼���ٶȼ�,(����������)

#define OFFSET_AV_NUM 	50					//У׼ƫ����ʱ��ƽ��������
#define FILTER_NUM 			10					//����ƽ���˲���ֵ����

#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//������Y����в���
#define FIX_GYRO_X 			1.02f				//������X����в���

#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת���ȣ���/180��

#define MAX_ACC  4096.0f						//+-8G		���ٶȼ�����
#define TO_DEG_S 500.0f      				//T = 2ms  Ĭ��Ϊ2ms ����ֵ����1/T

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 ITEMS ,
};


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
	int32_t filter_rate;									//�ٶ�
	float ecd_angle;											//�Ƕ�
}Encoder;



#endif

