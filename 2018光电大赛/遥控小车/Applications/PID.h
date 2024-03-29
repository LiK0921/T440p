#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

#include "mymath.h"

#include "filter.h"

/*=====================================================================================================================
						预估算法
=====================================================================================================================*/
typedef struct
{
	float fb_lim;				 //反馈限幅
	float value_old;		 //历史值
	float value;				 //估计输出（实际）
	float ut;					   //阶跃响应上升时间
	float fb_t;					 //反馈滞后时间
	
	float ct_out;        //控制器的输出量
	float feed_back;     //反馈值
	float fore_feed_back;//估计的反馈值
}forecast_t;


void forecast_calculate_I(float T,								//周期（单位：秒）
													forecast_t *forecast		//预估算法结构体
												 );		

void forecast_calculate_II(float T,								//周期（单位：秒）
													 forecast_t *forecast		//预估算法结构体
                          );		
													
													
/*=====================================================================================================================
						 *****
=====================================================================================================================*/
typedef struct
{
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd;		 	 //微分系数
	float k_pre_d; //previous_d 微分先行
	float inc_hz;  //不完全微分低通系数
	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 
	

}_PID_arg_st;


typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	float err_d;
	float err_d_lpf;
	float err_i;
	float ff;
	float pre_d;
  float differential;
  float out;
  float k_pre_d;
  
}_PID_val_st;

typedef struct PID_Regulator_t
{
	float ref;
	float exp;
	float fdb;
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float kp_offset;
	float ki_offset;
	float kd_offset;
}PID_Regulator_t;

float PID_calculate( float T,            //周期
										float in_ff,				//前馈
										float expect,				//期望值（设定值）
										float feedback,			//反馈值
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_lim			//integration limit，积分限幅
										   );			//输出
	
float My_PID_calculate( float T,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										float eliminate_I,
										_PID_arg_st *pid_arg, //PID参数结构体   argument count
										_PID_val_st *pid_val,	//PID数据结构体
										_PID_val_st *pid_val_temp,  //用于保存相关数据并将其发送至上位机进行观察
										float inte_lim			//integration limit，积分限幅
										   );
                    
#endif
