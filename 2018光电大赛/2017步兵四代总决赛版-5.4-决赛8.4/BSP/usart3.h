#ifndef __USART3_H
#define __USART3_H

#include "system.h"

extern bool Shoot_Allow;
extern u8 rxdata;

void usart3_Init(void);
void Sudoku_Data_Receive(void);

#endif

