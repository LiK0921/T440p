#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "include.h"

extern float GMPPosition_exp,GMPPosition_exp_filter;
extern _PID_val_st GMPPosition_Val;
extern float GMPASpeed,GMPASpeed_exp;   //��̨��ת���ٶ�
extern float PID_Out,GMPPosition_Out;
extern float GMPPosition_to_record,GMPPosition_exp_addrecord;

void GMYawControl_ParaInit(void);
void GMY_Val_Cal(float T);
void GMYawControlLoop(float T);
void GMYawControl_Out(void);
void Encoder_Protect(void);

#endif


