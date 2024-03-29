/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：PID.c
 * 描述    ：PID控制器
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "PID.h"

float PID_calculate( float T,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_lim			//integration limit，积分限幅
										 )	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);//范围在0~1之间
	//微分先行项，直接对被控量进行微分
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	pid_val->err =  (expect - feedback );
	//误差的微分项
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf );//低通滤波

	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d )*T;//)*T;
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);  //积分限幅
	
	out = pid_arg->k_ff *in_ff         //前馈项
	    + pid_arg->kp *pid_val->err    //比例项
	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential  //不完全微分项
    	+ pid_arg->ki *pid_val->err_i; //积分项
	
	pid_val->feedback_old = feedback;  
	pid_val->err_old = pid_val->err;
	
	return (out);
}

float My_PID_calculate( float T,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										float eliminate_I,
										_PID_arg_st *pid_arg, //PID参数结构体   argument count
										_PID_val_st *pid_val,	//PID数据结构体
										_PID_val_st *pid_val_temp,  //用于保存相关数据并将其发送至上位机进行观察
										float inte_lim			//integration limit，积分限幅
										 )	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	//保存观测
	pid_val_temp->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	pid_val->err =  (expect - feedback );
	
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	//保存观测
	pid_val_temp->differential = differential;
	
	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf );
	
	//保存观测
	pid_val_temp->err_d_lpf = pid_val->err_d_lpf;
	
//	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d )*T;//)*T;//
	//积分分离
	if(ABS(pid_val->err)<eliminate_I)
	{
		pid_val->err_i += (pid_val->err )*T*0.00003f;//)*T;//
		pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	}
	
	out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_arg->ki *pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
	
	pid_val_temp->err = pid_arg->kp *pid_val->err;
	pid_val_temp->err_d = pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential;
	pid_val_temp->err_i = pid_arg->ki *pid_val->err_i;
	pid_val_temp->ff = pid_arg->k_ff *in_ff;
	
	pid_val_temp->out = out;
	
	pid_val_temp->k_pre_d = pid_arg->k_pre_d;
	
	return (out);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
