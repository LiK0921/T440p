/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��PID.c
 * ����    ��PID������
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "PID.h"

float PID_calculate( float T,            //���ڣ���λ���룩
										float in_ff,				//ǰ��ֵ
										float expect,				//����ֵ���趨ֵ��
										float feedback,			//����ֵ����
										_PID_arg_st *pid_arg, //PID�����ṹ��
										_PID_val_st *pid_val,	//PID���ݽṹ��
										float inte_lim			//integration limit�������޷�
										 )	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);//��Χ��0~1֮��
	//΢�������ֱ�ӶԱ���������΢��
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	pid_val->err =  (expect - feedback );
	//����΢����
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf );//��ͨ�˲�

	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d )*T;//)*T;
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);  //�����޷�
	
	out = pid_arg->k_ff *in_ff         //ǰ����
	    + pid_arg->kp *pid_val->err    //������
	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential  //����ȫ΢����
    	+ pid_arg->ki *pid_val->err_i; //������
	
	pid_val->feedback_old = feedback;  
	pid_val->err_old = pid_val->err;
	
	return (out);
}

float My_PID_calculate( float T,            //���ڣ���λ���룩
										float in_ff,				//ǰ��ֵ
										float expect,				//����ֵ���趨ֵ��
										float feedback,			//����ֵ����
										float eliminate_I,
										_PID_arg_st *pid_arg, //PID�����ṹ��   argument count
										_PID_val_st *pid_val,	//PID���ݽṹ��
										_PID_val_st *pid_val_temp,  //���ڱ���������ݲ����䷢������λ�����й۲�
										float inte_lim			//integration limit�������޷�
										 )	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	//����۲�
	pid_val_temp->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);
	
	pid_val->err =  (expect - feedback );
	
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	//����۲�
	pid_val_temp->differential = differential;
	
	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf );
	
	//����۲�
	pid_val_temp->err_d_lpf = pid_val->err_d_lpf;
	
//	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d )*T;//)*T;//
	//���ַ���
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
