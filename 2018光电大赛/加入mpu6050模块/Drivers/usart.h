#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
#include "stdio.h"	
#include "stm32f4xx_conf.h"



extern u8 Rx_Buf[];
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Usart3_Init(u32 br_num);
void Usart3_IRQ(void);
void Usart3_Send(unsigned char *DataToSend ,u8 data_num);

#endif



