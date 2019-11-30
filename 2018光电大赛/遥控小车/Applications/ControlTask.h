#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "include.h"

#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.4f

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

extern float GMPPosition_exp,GMPPosition_exp_filter;
extern _PID_val_st GMPPosition_Val;
extern float GMPASpeed,GMPASpeed_exp;   //云台旋转角速度
extern float PID_Out,GMPPosition_Out;
extern float GMPPosition_to_record,GMPPosition_exp_addrecord;
extern PID_Regulator_t CM1Speed,CM2Speed,CM3Speed,CM4Speed;
extern float _temp;

void GMYawControl_ParaInit(void);
void GMY_Val_Cal(float T);
void GMYawControlLoop(float T);
void CMControl_PIDInit(void);
void CMControlLoop(float T);
void GMYawControl_Out(void);
void Encoder_Protect(void);

#endif


