#include "ControlTask.h"
#include "pid.h"
#include "parameter.h"
#include "mymath.h"
#include "filter.h"
#include "CanBusTask.h"
#include "pid.h"
#include "rc.h"
#include "task_mode.h"

_PID_val_st GMPPosition_Val;
_PID_arg_st GMPPosition_PID;

_PID_val_st GMPSpeed_Val;
_PID_arg_st GMPSpeed_PID;

float GMPPosition_to_record;

float GMPPosition_exp,GMPPosition_exp_filter,GMPPosition_exp_addrecord;
float GMPASpeed,GMPASpeed_exp;   //云台旋转角速度
float PID_Out,GMPPosition_Out;

void GMYawControl_ParaInit()
{
	GMPPosition_PID.kp = 1.0f * pid_setup.groups.pid_1.kp;
  GMPPosition_PID.kd = 0.1f * pid_setup.groups.pid_1.kd;
  GMPPosition_PID.ki = 0.1f * pid_setup.groups.pid_1.ki;
  GMPPosition_PID.k_pre_d = 0;
  GMPPosition_PID.inc_hz = 0;
  GMPPosition_PID.k_inc_d_norm = 0;  
	
	GMPSpeed_PID.kp=10.0f * pid_setup.groups.pid_2.kp;
  GMPSpeed_PID.ki=0.1f * pid_setup.groups.pid_2.ki;
  GMPSpeed_PID.kd=0;
  GMPSpeed_PID.k_pre_d=1.0f * pid_setup.groups.pid_2.kd; 
  GMPSpeed_PID.inc_hz=0; 
  GMPSpeed_PID.k_inc_d_norm=0;
  GMPSpeed_PID.k_ff=0.1f;
}

void GMY_Val_Cal(float T)
{
  GMPPosition_exp = 10.0f * pid_setup.groups.pid_17.kp;
	GMPPosition_exp = To_180_degrees(GMPPosition_exp);
	//GMPPosition_exp_filter +=  0.01f * pid_setup.groups.pid_17.ki *3.14f *( GMPPosition_exp - GMPPosition_exp_filter );
	LPF_1_(1.0f * pid_setup.groups.pid_17.ki,T,GMPPosition_exp,GMPPosition_exp_filter);
	if(state_prepare == 1)
	{
		GMPPosition_exp_addrecord = GMPPosition_to_record + GMPPosition_exp_filter;
	}		
}

//云台pitch轴控制程序
void GMYawControlLoop(float T)
{
	float out,differential;
	
	//位置环PID
	GMPPosition_PID.k_inc_d_norm = LIMIT(GMPPosition_PID.k_inc_d_norm,0,1);//范围在0~1之间
	//微分先行项，直接对被控量进行微分
	GMPPosition_Val.feedback_d = (-1.0f) *(GMPPosition_exp_addrecord - GMPPosition_Val.feedback_old) *safe_div(1.0f,T,0);
	
	GMPPosition_Val.err = To_180_degrees(GMPPosition_exp_addrecord - CM2Encoder.ecd_angle);
	//误差的微分项
	GMPPosition_Val.err_d = (GMPPosition_Val.err - GMPPosition_Val.err_old) *safe_div(1.0f,T,0);
	
	differential = (GMPPosition_PID.kd *GMPPosition_Val.err_d + GMPPosition_PID.k_pre_d *GMPPosition_Val.feedback_d);
	
	LPF_1_(GMPPosition_PID.inc_hz,T,differential,GMPPosition_Val.err_d_lpf );//低通滤波

	GMPPosition_Val.err_i += (GMPPosition_Val.err + GMPPosition_PID.k_pre_d *GMPPosition_Val.feedback_d )*T;//)*T;
	GMPPosition_Val.err_i = LIMIT(GMPSpeed_Val.err_i,-50,50);  //积分限幅
	
	GMPPosition_Out = 	GMPPosition_PID.kp *GMPPosition_Val.err    //比例项
				+ GMPPosition_PID.k_inc_d_norm *GMPSpeed_Val.err_d_lpf + (1.0f-GMPPosition_PID.k_inc_d_norm) *differential  //不完全微分项
				+ GMPPosition_PID.ki *GMPPosition_Val.err_i; //积分项
	
	if(mode_state == 0)
	{
		GMPASpeed_exp = CH_filter[ROL];
	}
	else if(mode_state == 1)
	{
		//速度环PID
		GMPASpeed_exp = GMPPosition_Out;
	}
	
	
  GMPASpeed = CM2Encoder.filter_rate;

	GMPSpeed_PID.k_inc_d_norm = LIMIT(GMPSpeed_PID.k_inc_d_norm,0,1);//范围在0~1之间
	//微分先行项，直接对被控量进行微分
	GMPSpeed_Val.feedback_d = (-1.0f) *(GMPASpeed - GMPSpeed_Val.feedback_old) *safe_div(1.0f,T,0);
	
	GMPSpeed_Val.err =  (GMPASpeed_exp - GMPASpeed );
	//误差的微分项
	GMPSpeed_Val.err_d = (GMPSpeed_Val.err - GMPSpeed_Val.err_old) *safe_div(1.0f,T,0);
	
	differential = (GMPSpeed_PID.kd *GMPSpeed_Val.err_d + GMPSpeed_PID.k_pre_d *GMPSpeed_Val.feedback_d);
	
	LPF_1_(GMPSpeed_PID.inc_hz,T,differential,GMPSpeed_Val.err_d_lpf );//低通滤波

	GMPSpeed_Val.err_i += (GMPSpeed_Val.err + GMPSpeed_PID.k_pre_d *GMPSpeed_Val.feedback_d )*T;//)*T;
	GMPSpeed_Val.err_i = LIMIT(GMPSpeed_Val.err_i,-50,50);  //积分限幅
	
	out = 	GMPSpeed_PID.kp *GMPSpeed_Val.err    //比例项
				+ GMPSpeed_PID.k_inc_d_norm *GMPSpeed_Val.err_d_lpf + (1.0f-GMPSpeed_PID.k_inc_d_norm) *differential  //不完全微分项
				+ GMPSpeed_PID.ki *GMPSpeed_Val.err_i; //积分项
	
	
	GMPSpeed_Val.feedback_old = GMPASpeed;  
	GMPSpeed_Val.err_old = GMPSpeed_Val.err;
	
	GMPPosition_Val.feedback_old = CM2Encoder.ecd_angle;
	GMPPosition_Val.err_old = GMPPosition_Val.err;
	
	if(mode_state == 2)
	{
		Set_CM_Speed(CAN1,0,0,0,0);
	}
	else
		Set_CM_Speed(CAN1,0,(s16)(out),0,0);
}

void GMYawControl_Out()
{
   ;
}

void Encoder_Err()
{
	;
}


