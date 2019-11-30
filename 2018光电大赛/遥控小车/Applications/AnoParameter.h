#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "stm32f4xx.h"

extern union Parameter Ano_Parame;

typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
}_parameter_state_st ;
extern _parameter_state_st para_sta;
extern u8 write_flag;

void Ano_Parame_Read(void);
void Ano_Parame_Write_task(u16 dT_ms);
void PID_Rest(void);
void Parame_Reset(void);
void All_PID_Init(void);

#endif 

