#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "include.h"

extern float GMPPosition_exp,GMPPosition_exp_filter;
extern _PID_val_st GMPPosition_Val;

void GMYawControl_ParaInit(void);
float GMPPosition_err_cal(float err);
void GMYawControlLoop(float T);
void GMYawControl_Out(void);


#endif


