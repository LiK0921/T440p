#ifndef __UART5_H
#define __UART5_H

#include "system.h"

void uart5_Init(void);
void Sudoku_Data_Receive(void);

void UART5_Init(u32 bound);
void UART5_sendDataToPrint(short dax,short day,short daz,short dgx,short dgy,short dgz,short dmx, short my,short dmz);
#endif

