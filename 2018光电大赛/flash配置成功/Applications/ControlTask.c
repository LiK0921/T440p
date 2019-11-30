#include "ControlTask.h"
#include "pid.h"
#include "parameter.h"
#include "mymath.h"
#include "filter.h"
#include "CanBusTask.h"
#include "pid.h"

_PID_val_st GMPPosition_Val;
_PID_arg_st GMPPosition_PID;

_PID_val_st GMPSpeed_Val;
_PID_arg_st GMPSpeed_PID;


float GMPPosition_exp,GMPPosition_exp_filter;
float GMPASpeed;   //云台旋转角速度

void GMYawControl_ParaInit()
{
	GMPPosition_PID.kp = 1.0f * pid_setup.groups.pid_1.kp;
  GMPPosition_PID.kd = 0.1f * pid_setup.groups.pid_1.kd;
  GMPPosition_PID.ki = 0.1f * pid_setup.groups.pid_1.ki;
  GMPPosition_PID.k_pre_d = 0;
  GMPPosition_PID.inc_hz = 0;
  GMPPosition_PID.k_inc_d_norm = 0;  
	
	GMPSpeed_PID.kp=1.0f * pid_setup.groups.pid_2.kp;
  GMPSpeed_PID.ki=0.1f * pid_setup.groups.pid_2.ki;
  GMPSpeed_PID.kd=0;
  GMPSpeed_PID.k_pre_d=1.0f * pid_setup.groups.pid_2.kd; 
  GMPSpeed_PID.inc_hz=0; 
  GMPSpeed_PID.k_inc_d_norm=0;
  GMPSpeed_PID.k_ff=0.1f;
}

float GMPPosition_err_cal(float err)
{
	return (err<-300?(err+360):err);
}

//云台pitch轴控制程序
void GMYawControlLoop(float T)
{
	GMPPosition_exp = 10.0f * pid_setup.groups.pid_17.kp;
	GMPPosition_exp = To_180_degrees(GMPPosition_exp);
	GMPPosition_exp_filter +=  0.03f *3.14f *( GMPPosition_exp - GMPPosition_exp_filter );
	GMPPosition_Val.err = To_180_degrees(GMPPosition_exp_filter - CM2Encoder.ecd_angle);
	

}

void GMYawControl_Out()
{
   ;
}


