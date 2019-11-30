#include "infrared.h"

/*-GPIO--(PB14---TIM2)--*/

#define INFRARED_CODE_LENGTH 8	// ���봮����
int8_t infrared_code[INFRARED_CODE_LENGTH]={	// ���봮��
	2, 2, 2, 2, 2, 2, 2, 2											// 8��������
};
int8_t code_i=0;	// ��¼��ǰ���͵������������봮�е����
int8_t code_flag=0;		// ��¼��ǰ���͵�״̬��
// 0��׼���������봮��
// 1���Զ��������ӣ��������ӵ�ֵ������
// 2������������ʱ�䵽��ֹͣ����2ms������
// 3��8�������Ӽ�2msֹͣ������ϣ����봮������������code_flag����Ϊ0

void Infrared_GPIO_Init(void)
{
   
	GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB,ENABLE);
  
	gpio.GPIO_Pin = GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_14);
}

void Infrared_Init(void)
{
	Infrared_GPIO_Init();		// ��ʼ��IO�ڣ�IO���õ�ʱ������������ߵ�ƽ��������
	TIM2_Init();						// ��ʼ��TIM2�����ڿ��ƺ��ⷢ��Ƶ��,ʹ֮�ܱ�����
	TIM5_Init();						// ��ʼ��TIM5�����ڿ��ƺ��ⷢ�䡰���롱
}

void Infrared_On(void)
{
//	int i;
//	for(i=0;i<8;i++)
//	{
//		TIM2_Start();
////		delayMicroseconds(code[i]);
//		delay(2);
//		TIM_Cmd(TIM2, DISABLE);
//		GPIO_SetBits(GPIOB, GPIO_Pin_14);
////		delayMicroseconds(192);
//		delay(2);
//	}
////	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	
	code_i = 0;
	code_flag = 0;
	TIM5_Start();
}

void Infrared_Off(void)
{
	TIM_Cmd(TIM5, DISABLE);
	GPIO_SetBits(GPIOB, GPIO_Pin_14);
}


/***********************************************************************/
/*                        TIM2				                                 */
/***********************************************************************/
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef  tim;
	NVIC_InitTypeDef         nvic;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2  ,ENABLE);
	
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	tim.TIM_Prescaler = 21-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_Period =104;//52;
	TIM_TimeBaseInit(TIM2,&tim);
}

void TIM2_Start(void)
{
	TIM_Cmd(TIM2, ENABLE);	 
	TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
}

void TIM2_IRQHandler(void)  
{
  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		
		GPIOB->ODR ^= GPIO_Pin_14;
  }
}

/***********************************************************************/
/*                        TIM5				                                 */
/***********************************************************************/
void TIM5_Init(void)
{
	TIM_TimeBaseInitTypeDef  tim;
	NVIC_InitTypeDef         nvic;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5  ,ENABLE);
	
	nvic.NVIC_IRQChannel = TIM5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	tim.TIM_Prescaler = 1680-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_Period = 49;	//1ms
	TIM_TimeBaseInit(TIM5,&tim);
}

void TIM5_Start(void)
{
	TIM_Cmd(TIM5, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_Update,ENABLE);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
}

int8_t INFRARED_CODE_INTERVAL=50;	// ���봮���ͼ��������Ч���������趨����
void TIM5_IRQHandler(void)
{
	static int8_t ms_target=0;
	static int8_t ms_count=0;
	
  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		
		
		if(code_flag == 0)					// ׼�������µ����봮 
		{
			TIM2_Start();														// ���������ź�
			code_flag = 1;
			ms_count = 0;
			ms_target = infrared_code[code_i];			// ���������ӵ�ֵ���м���
		}
		else
		{
			ms_count++;
		}
		
		if(ms_count == ms_target && ms_target != 0)
		{
			if(code_flag == 1)				// ������ʱ���� 
			{
				TIM_Cmd(TIM2, DISABLE);								// ֹͣ�������
				GPIO_SetBits(GPIOB, GPIO_Pin_14);			// �رպ��ⷢ��
				ms_count = 0;
				ms_target = 2;
				code_flag = 2;
			}
			else if(code_flag == 2)		// ֹͣ��ʱ���� 
			{
				code_i++;				// ���1�������Ӽ�2msֹͣ�������
				if(code_i == INFRARED_CODE_LENGTH)	// 1�����뷢�����
				{
					ms_count = 0;
					ms_target = INFRARED_CODE_INTERVAL;
					code_flag = 3;
				}
				else
				{
					TIM2_Start();												// ���������ź�
					code_flag = 1;
					ms_count = 0;
					ms_target = infrared_code[code_i];	// ���������ӵ�ֵ���м���
				}
			}
			else if(code_flag == 3)		// ���봮�������
			{
				code_i = 0;			// �����Ӽ�������
				ms_count = 0;
				ms_target = 0;
				code_flag = 0;	// Ϊ��һ�����봮������׼��
			}
		}
  }
}
