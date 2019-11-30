#include "infrared.h"

/*-GPIO--(PB14---TIM2)--*/

#define INFRARED_CODE_LENGTH 8	// 密码串长度
int8_t infrared_code[INFRARED_CODE_LENGTH]={	// 密码串：
	2, 2, 2, 2, 2, 2, 2, 2											// 8个密码子
};
int8_t code_i=0;	// 记录当前发送的密码子在密码串中的序号
int8_t code_flag=0;		// 记录当前发送的状态：
// 0、准备发送密码串；
// 1、以读入密码子，按密码子的值计数；
// 2、发送密码子时间到，停止发射2ms计数；
// 3、8个密码子及2ms停止发送完毕，密码串间隔计数，完毕code_flag重置为0

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
	Infrared_GPIO_Init();		// 初始化IO口，IO口置低时板子引脚输出高电平，红外亮
	TIM2_Init();						// 初始化TIM2，用于控制红外发光频率,使之能被接收
	TIM5_Init();						// 初始化TIM5，用于控制红外发射“密码”
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

int8_t INFRARED_CODE_INTERVAL=50;	// 密码串发送间隔，根据效果和需求设定……
void TIM5_IRQHandler(void)
{
	static int8_t ms_target=0;
	static int8_t ms_count=0;
	
  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		
		
		if(code_flag == 0)					// 准备发送新的密码串 
		{
			TIM2_Start();														// 发射脉冲信号
			code_flag = 1;
			ms_count = 0;
			ms_target = infrared_code[code_i];			// 读入密码子的值进行计数
		}
		else
		{
			ms_count++;
		}
		
		if(ms_count == ms_target && ms_target != 0)
		{
			if(code_flag == 1)				// 发射延时结束 
			{
				TIM_Cmd(TIM2, DISABLE);								// 停止输出脉冲
				GPIO_SetBits(GPIOB, GPIO_Pin_14);			// 关闭红外发射
				ms_count = 0;
				ms_target = 2;
				code_flag = 2;
			}
			else if(code_flag == 2)		// 停止延时结束 
			{
				code_i++;				// 标记1个密码子及2ms停止发送完成
				if(code_i == INFRARED_CODE_LENGTH)	// 1串密码发送完成
				{
					ms_count = 0;
					ms_target = INFRARED_CODE_INTERVAL;
					code_flag = 3;
				}
				else
				{
					TIM2_Start();												// 发射脉冲信号
					code_flag = 1;
					ms_count = 0;
					ms_target = infrared_code[code_i];	// 读入密码子的值进行计数
				}
			}
			else if(code_flag == 3)		// 密码串间隔结束
			{
				code_i = 0;			// 密码子计数清零
				ms_count = 0;
				ms_target = 0;
				code_flag = 0;	// 为新一次密码串发送做准备
			}
		}
  }
}
