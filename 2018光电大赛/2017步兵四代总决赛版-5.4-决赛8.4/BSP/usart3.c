#include "usart3.h"

void usart3_Init()						
{
	USART_InitTypeDef uart3;
	GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
	gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&gpio);

	uart3.USART_BaudRate = 115200;       
	uart3.USART_WordLength = USART_WordLength_8b;
	uart3.USART_StopBits = USART_StopBits_1;
	uart3.USART_Parity = USART_Parity_No;
	uart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  uart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&uart3);

  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART3,ENABLE);
    
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 3;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

int16_t sensor_num;
extern unsigned char sensor_data;
void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(USART3);
	}
	//接收中断 (接收寄存器非空) 
	if(USART3->SR & (1<<5))    
	{		
		sensor_data = USART3->DR;	
	  sensor_Data_handle(sensor_data);
		sensor_num=0;
	}
}

