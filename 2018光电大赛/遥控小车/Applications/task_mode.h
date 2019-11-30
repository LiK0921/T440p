#ifndef __FLY_MODE_H
#define __FLY_MODE_H

#include "stm32f4xx.h"
#include "include.h"
#include "parameter.h"

enum
{
	BARO=0,
	GPS,
	BACK_HOME,
	//UTRASONIC,

};

extern u8 mode_value[],mode_state;
extern u8 auto_fly_flag,auto_fly_count;
extern u8 task_on,state_prepare;

void mode_check(float *ch_in,u8 *mode_value);
#endif
