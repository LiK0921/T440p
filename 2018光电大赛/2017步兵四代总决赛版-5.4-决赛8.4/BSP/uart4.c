#include "uart4.h"

void uart4_Init(void)    
{
  USART_InitTypeDef uart4;
	GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); 
	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA,&gpio);

	uart4.USART_BaudRate = 115200;   //裁判系统波特率115200
	uart4.USART_WordLength = USART_WordLength_8b;
	uart4.USART_StopBits = USART_StopBits_1;
	uart4.USART_Parity = USART_Parity_No;
	uart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART4,&uart4);
	
	USART_Cmd(UART4,ENABLE); 
	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);  
	
	nvic.NVIC_IRQChannel = UART4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 3;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic); 
}

int Judge_num;
void UART4_IRQHandler(void)
{
		if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) != RESET)
		{
			USART_ReceiveData(UART4);
		}		
		if(UART4->SR & (1<<5))    
		{
				Judge_data = UART4->DR;
				if(Judge_data==0xA5)
				{	
						data_num=0;
						Judge_Rx_Buf[data_num]=Judge_data;
				}
				else
				{
						Judge_Rx_Buf[data_num]=Judge_data;
				}
				data_num++;		
				if(data_num>99)
				{
						data_num=0;	
				}		
				Judge_num=0;				
		}
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);  
}
