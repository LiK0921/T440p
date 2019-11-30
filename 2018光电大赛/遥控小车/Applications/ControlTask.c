#include "ControlTask.h"
#include "pid.h"
#include "parameter.h"
#include "mymath.h"
#include "filter.h"
#include "CanBusTask.h"
#include "pid.h"
#include "rc.h"
#include "task_mode.h"
#include "pid.h"

_PID_val_st GMPPosition_Val;
_PID_arg_st GMPPosition_PID;

_PID_val_st GMPSpeed_Val;
_PID_arg_st GMPSpeed_PID;

ChassisSpeed_Ref_t ChassisSpeedRef;

float GMPPosition_to_record;

float GMPPosition_exp,GMPPosition_exp_filter,GMPPosition_exp_addrecord;
float GMPASpeed,GMPASpeed_exp;   //��̨��ת���ٶ�
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

//��̨pitch����Ƴ���
void GMYawControlLoop(float T)
{
	float out,differential;
	
	//λ�û�PID
	GMPPosition_PID.k_inc_d_norm = LIMIT(GMPPosition_PID.k_inc_d_norm,0,1);//��Χ��0~1֮��
	//΢�������ֱ�ӶԱ���������΢��
	GMPPosition_Val.feedback_d = (-1.0f) *(GMPPosition_exp_addrecord - GMPPosition_Val.feedback_old) *safe_div(1.0f,T,0);
	
	GMPPosition_Val.err = To_180_degrees(GMPPosition_exp_addrecord - CM2Encoder.ecd_angle);
	//����΢����
	GMPPosition_Val.err_d = (GMPPosition_Val.err - GMPPosition_Val.err_old) *safe_div(1.0f,T,0);
	
	differential = (GMPPosition_PID.kd *GMPPosition_Val.err_d + GMPPosition_PID.k_pre_d *GMPPosition_Val.feedback_d);
	
	LPF_1_(GMPPosition_PID.inc_hz,T,differential,GMPPosition_Val.err_d_lpf );//��ͨ�˲�

	GMPPosition_Val.err_i += (GMPPosition_Val.err + GMPPosition_PID.k_pre_d *GMPPosition_Val.feedback_d )*T;//)*T;
	GMPPosition_Val.err_i = LIMIT(GMPSpeed_Val.err_i,-50,50);  //�����޷�
	
	GMPPosition_Out = 	GMPPosition_PID.kp *GMPPosition_Val.err    //������
				+ GMPPosition_PID.k_inc_d_norm *GMPSpeed_Val.err_d_lpf + (1.0f-GMPPosition_PID.k_inc_d_norm) *differential  //����ȫ΢����
				+ GMPPosition_PID.ki *GMPPosition_Val.err_i; //������
	
	if(mode_state == 0)
	{
		GMPASpeed_exp = CH_filter[ROL];
	}
	else if(mode_state == 1)
	{
		//�ٶȻ�PID
		GMPASpeed_exp = GMPPosition_Out;
	}
	
	
  GMPASpeed = CM2Encoder.filter_rate;

	GMPSpeed_PID.k_inc_d_norm = LIMIT(GMPSpeed_PID.k_inc_d_norm,0,1);//��Χ��0~1֮��
	//΢�������ֱ�ӶԱ���������΢��
	GMPSpeed_Val.feedback_d = (-1.0f) *(GMPASpeed - GMPSpeed_Val.feedback_old) *safe_div(1.0f,T,0);
	
	GMPSpeed_Val.err =  (GMPASpeed_exp - GMPASpeed );
	//����΢����
	GMPSpeed_Val.err_d = (GMPSpeed_Val.err - GMPSpeed_Val.err_old) *safe_div(1.0f,T,0);
	
	differential = (GMPSpeed_PID.kd *GMPSpeed_Val.err_d + GMPSpeed_PID.k_pre_d *GMPSpeed_Val.feedback_d);
	
	LPF_1_(GMPSpeed_PID.inc_hz,T,differential,GMPSpeed_Val.err_d_lpf );//��ͨ�˲�

	GMPSpeed_Val.err_i += (GMPSpeed_Val.err + GMPSpeed_PID.k_pre_d *GMPSpeed_Val.feedback_d )*T;//)*T;
	GMPSpeed_Val.err_i = LIMIT(GMPSpeed_Val.err_i,-50,50);  //�����޷�
	
	out = 	GMPSpeed_PID.kp *GMPSpeed_Val.err    //������
				+ GMPSpeed_PID.k_inc_d_norm *GMPSpeed_Val.err_d_lpf + (1.0f-GMPSpeed_PID.k_inc_d_norm) *differential  //����ȫ΢����
				+ GMPSpeed_PID.ki *GMPSpeed_Val.err_i; //������
	
	
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

PID_Regulator_t CM1Speed,CM2Speed,CM3Speed,CM4Speed;

_PID_val_st CM1SpeedVAL,CM2SpeedVAL,CM3SpeedVAL,CM4SpeedVAL;
_PID_arg_st CM1SpeedPID,CM2SpeedPID,CM3SpeedPID,CM4SpeedPID;

void CMControl_PIDInit(void)
{
	CM1SpeedPID.kp=10.0f * pid_setup.groups.pid_3.kp;
  CM1SpeedPID.ki=0.1f * pid_setup.groups.pid_3.ki;
  CM1SpeedPID.kd=0;
  CM1SpeedPID.k_pre_d=1.0f * pid_setup.groups.pid_3.kd; 
  CM1SpeedPID.inc_hz=0; 
  CM1SpeedPID.k_inc_d_norm=0;
  CM1SpeedPID.k_ff=0;
	
	CM2SpeedPID.kp=10.0f * pid_setup.groups.pid_3.kp;
  CM2SpeedPID.ki=0.1f * pid_setup.groups.pid_3.ki;
  CM2SpeedPID.kd=0;
  CM2SpeedPID.k_pre_d=1.0f * pid_setup.groups.pid_3.kd; 
  CM2SpeedPID.inc_hz=0; 
  CM2SpeedPID.k_inc_d_norm=0;
  CM2SpeedPID.k_ff=0;
	
	CM3SpeedPID.kp=10.0f * pid_setup.groups.pid_3.kp;
  CM3SpeedPID.ki=0.1f * pid_setup.groups.pid_3.ki;
  CM3SpeedPID.kd=0;
  CM3SpeedPID.k_pre_d=1.0f * pid_setup.groups.pid_3.kd; 
  CM3SpeedPID.inc_hz=0; 
  CM3SpeedPID.k_inc_d_norm=0;
  CM3SpeedPID.k_ff=0;
	
	CM4SpeedPID.kp=10.0f * pid_setup.groups.pid_3.kp;
  CM4SpeedPID.ki=0.1f * pid_setup.groups.pid_3.ki;
  CM4SpeedPID.kd=0;
  CM4SpeedPID.k_pre_d=1.0f * pid_setup.groups.pid_3.kd; 
  CM4SpeedPID.inc_hz=0; 
  CM4SpeedPID.k_inc_d_norm=0;
  CM4SpeedPID.k_ff=0;
}

float Max_speed,Max_CMspeed;
//���̿�������
void CMControlLoop(float T)
{  
	float CM1_differential,CM2_differential,CM3_differential,CM4_differential;
	ChassisSpeedRef.forward_back_ref = CH_filter[PIT] * STICK_TO_CHASSIS_SPEED_REF_FACT;
  ChassisSpeedRef.left_right_ref   = CH_filter[ROL] * STICK_TO_CHASSIS_SPEED_REF_FACT; 
	ChassisSpeedRef.rotate_ref       = CH_filter[YAW] * STICK_TO_CHASSIS_SPEED_REF_FACT;
	//������ת������
//	if(GetWorkState()==PREPARE_STATE) //�����׶Σ����̲���ת
//	{
//		ChassisSpeedRef.rotate_ref = 0;	 
//	}
//	else
//	{
//		 //���̸����������תPID����
//		 CMRotatePID.ref = 0;
//		 CMRotatePID.fdb = GMYawEncoder.ecd_angle;
//		 CMRotatePID.Calc(&CMRotatePID);   
//		 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
//	}
//	if(NS == 0)      //���ң������ʧ��ǿ�ƽ��ٶ��趨ֵreset
//	{
//		ChassisSpeedRef.forward_back_ref = 0;
//		ChassisSpeedRef.left_right_ref = 0;
//	}
  Max_speed = ABS_MAX(ABS(ChassisSpeedRef.forward_back_ref),ABS(ChassisSpeedRef.left_right_ref),ABS(ChassisSpeedRef.rotate_ref));
	
	CM1Speed.ref = ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
	CM2Speed.ref = -ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
  CM3Speed.ref = -ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
	CM4Speed.ref = ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref;
	
	Max_CMspeed = _MAX(ABS_MAX(ABS(CM1Speed.ref),ABS(CM2Speed.ref),ABS(CM3Speed.ref)),ABS(CM4Speed.ref));
	
	CM1Speed.exp = safe_div(CM1Speed.ref,Max_CMspeed,0)*Max_speed;
	CM2Speed.exp = safe_div(CM2Speed.ref,Max_CMspeed,0)*Max_speed;
	CM3Speed.exp = safe_div(CM3Speed.ref,Max_CMspeed,0)*Max_speed;
	CM4Speed.exp = safe_div(CM4Speed.ref,Max_CMspeed,0)*Max_speed;
  
	CM1Speed.fdb = CM1Encoder.filter_rate;
	CM2Speed.fdb = CM2Encoder.filter_rate;
	CM3Speed.fdb = CM3Encoder.filter_rate;
	CM4Speed.fdb = CM4Encoder.filter_rate;
	
//	CM1SpeedPID.Calc(&CM1SpeedPID);
//	CM2SpeedPID.Calc(&CM2SpeedPID);
//	CM3SpeedPID.Calc(&CM3SpeedPID);
//	CM4SpeedPID.Calc(&CM4SpeedPID);
	
	  //CM1Speed.output = PID_calculate(T,0,CM1Speed.ref,CM1Speed.fdb,&CM1SpeedPID,&CM1SpeedVAL,50.0f);
		
		CM1SpeedPID.k_inc_d_norm = LIMIT(CM1SpeedPID.k_inc_d_norm,0,1);//��Χ��0~1֮��
		//΢�������ֱ�ӶԱ���������΢��
		CM1SpeedVAL.feedback_d = (-1.0f) *(CM1Speed.fdb - CM1SpeedVAL.feedback_old) *safe_div(1.0f,T,0);
		
		CM1SpeedVAL.err =  (CM1Speed.exp - CM1Speed.fdb );
		//����΢����
		CM1SpeedVAL.err_d = (CM1SpeedVAL.err - CM1SpeedVAL.err_old) *safe_div(1.0f,T,0);
		
		CM1_differential = (CM1SpeedPID.kd *CM1SpeedVAL.err_d + CM1SpeedPID.k_pre_d *CM1SpeedVAL.feedback_d);
		
		LPF_1_(CM1SpeedPID.inc_hz,T,CM1_differential,CM1SpeedVAL.err_d_lpf );//��ͨ�˲�

		CM1SpeedVAL.err_i += (CM1SpeedVAL.err + CM1SpeedPID.k_pre_d *CM1SpeedVAL.feedback_d )*T;//)*T;
		CM1SpeedVAL.err_i = LIMIT(CM1SpeedVAL.err_i,-50,50);  //�����޷�
		
		CM1Speed.output = 	CM1SpeedPID.kp *CM1SpeedVAL.err    //������
											+ CM1SpeedPID.k_inc_d_norm *CM1SpeedVAL.err_d_lpf + (1.0f-CM1SpeedPID.k_inc_d_norm) *CM1_differential  //����ȫ΢����
											+ CM1SpeedPID.ki *CM1SpeedVAL.err_i; //������
					
		CM2SpeedPID.k_inc_d_norm = LIMIT(CM2SpeedPID.k_inc_d_norm,0,1);//��Χ��0~1֮��
		//΢�������ֱ�ӶԱ���������΢��
		CM2SpeedVAL.feedback_d = (-1.0f) *(CM2Speed.fdb - CM2SpeedVAL.feedback_old) *safe_div(1.0f,T,0);
		
		CM2SpeedVAL.err =  (CM2Speed.exp - CM2Speed.fdb );
		//����΢����
		CM2SpeedVAL.err_d = (CM2SpeedVAL.err - CM2SpeedVAL.err_old) *safe_div(1.0f,T,0);
		
		CM2_differential = (CM2SpeedPID.kd *CM2SpeedVAL.err_d + CM2SpeedPID.k_pre_d *CM2SpeedVAL.feedback_d);
		
		LPF_1_(CM2SpeedPID.inc_hz,T,CM2_differential,CM2SpeedVAL.err_d_lpf );//��ͨ�˲�

		CM2SpeedVAL.err_i += (CM2SpeedVAL.err + CM2SpeedPID.k_pre_d *CM2SpeedVAL.feedback_d )*T;//)*T;
		CM2SpeedVAL.err_i = LIMIT(CM2SpeedVAL.err_i,-50,50);  //�����޷�
		
		CM2Speed.output = 	CM2SpeedPID.kp *CM2SpeedVAL.err    //������
					+ CM2SpeedPID.k_inc_d_norm *CM2SpeedVAL.err_d_lpf + (1.0f-CM2SpeedPID.k_inc_d_norm) *CM2_differential  //����ȫ΢����
					+ CM2SpeedPID.ki *CM2SpeedVAL.err_i; //������
					
		CM3SpeedPID.k_inc_d_norm = LIMIT(CM3SpeedPID.k_inc_d_norm,0,1);//��Χ��0~1֮��
		//΢�������ֱ�ӶԱ���������΢��
		CM3SpeedVAL.feedback_d = (-1.0f) *(CM3Speed.fdb - CM3SpeedVAL.feedback_old) *safe_div(1.0f,T,0);
		
		CM3SpeedVAL.err =  (CM3Speed.exp - CM3Speed.fdb );
		//����΢����
		CM3SpeedVAL.err_d = (CM3SpeedVAL.err - CM3SpeedVAL.err_old) *safe_div(1.0f,T,0);
		
		CM3_differential = (CM3SpeedPID.kd *CM3SpeedVAL.err_d + CM3SpeedPID.k_pre_d *CM3SpeedVAL.feedback_d);
		
		LPF_1_(CM3SpeedPID.inc_hz,T,CM3_differential,CM3SpeedVAL.err_d_lpf );//��ͨ�˲�

		CM3SpeedVAL.err_i += (CM3SpeedVAL.err + CM3SpeedPID.k_pre_d *CM3SpeedVAL.feedback_d )*T;//)*T;
		CM3SpeedVAL.err_i = LIMIT(CM3SpeedVAL.err_i,-50,50);  //�����޷�
		
		CM3Speed.output = 	CM3SpeedPID.kp *CM3SpeedVAL.err    //������
					+ CM3SpeedPID.k_inc_d_norm *CM3SpeedVAL.err_d_lpf + (1.0f-CM3SpeedPID.k_inc_d_norm) *CM3_differential  //����ȫ΢����
					+ CM3SpeedPID.ki *CM3SpeedVAL.err_i; //������
					
		CM4SpeedPID.k_inc_d_norm = LIMIT(CM4SpeedPID.k_inc_d_norm,0,1);//��Χ��0~1֮��
		//΢�������ֱ�ӶԱ���������΢��
		CM4SpeedVAL.feedback_d = (-1.0f) *(CM4Speed.fdb - CM4SpeedVAL.feedback_old) *safe_div(1.0f,T,0);
		
		CM4SpeedVAL.err =  (CM4Speed.exp - CM4Speed.fdb );
		//����΢����
		CM4SpeedVAL.err_d = (CM4SpeedVAL.err - CM4SpeedVAL.err_old) *safe_div(1.0f,T,0);
		
		CM4_differential = (CM4SpeedPID.kd *CM4SpeedVAL.err_d + CM4SpeedPID.k_pre_d *CM4SpeedVAL.feedback_d);
		
		LPF_1_(CM4SpeedPID.inc_hz,T,CM4_differential,CM4SpeedVAL.err_d_lpf );//��ͨ�˲�

		CM4SpeedVAL.err_i += (CM4SpeedVAL.err + CM4SpeedPID.k_pre_d *CM4SpeedVAL.feedback_d )*T;//)*T;
		CM4SpeedVAL.err_i = LIMIT(CM4SpeedVAL.err_i,-50,50);  //�����޷�
		
		CM4Speed.output = 	CM4SpeedPID.kp *CM4SpeedVAL.err    //������
					+ CM4SpeedPID.k_inc_d_norm *CM4SpeedVAL.err_d_lpf + (1.0f-CM4SpeedPID.k_inc_d_norm) *CM4_differential  //����ȫ΢����
					+ CM4SpeedPID.ki *CM4SpeedVAL.err_i; //������
//	CM2Speed.output = PID_calculate(T,0,CM2Speed.ref,CM2Speed.fdb,&CM2SpeedPID,&CM2SpeedVAL,50.0f);
//	CM3Speed.output = PID_calculate(T,0,CM3Speed.ref,CM3Speed.fdb,&CM3SpeedPID,&CM3SpeedVAL,50.0f);
//	CM4Speed.output = PID_calculate(T,0,CM4Speed.ref,CM4Speed.fdb,&CM4SpeedPID,&CM4SpeedVAL,50.0f);
	 
	 CM1SpeedVAL.feedback_old = CM1Speed.fdb;  
	 CM1SpeedVAL.err_old = CM1SpeedVAL.err;
	 CM2SpeedVAL.feedback_old = CM2Speed.fdb;  
	 CM2SpeedVAL.err_old = CM2SpeedVAL.err;
	 CM3SpeedVAL.feedback_old = CM3Speed.fdb;  
	 CM3SpeedVAL.err_old = CM3SpeedVAL.err;
	 CM4SpeedVAL.feedback_old = CM4Speed.fdb;  
	 CM4SpeedVAL.err_old = CM4SpeedVAL.err;
	 
	 if(mode_state == 2)    //|| dead_lock_flag == 1����ͣ����������У׼���޿�������ʱ����ʹ���̿���ֹͣ
	 {
		 Set_CM_Speed(CAN1, 0,0,0,0);
	 }
	 else
	 {
		 Set_CM_Speed(CAN1,CM1Speed.output,CM2Speed.output,CM3Speed.output,CM4Speed.output);	
		   //Set_CM_Speed(CAN1,CM1Speed.output,0,0,0);	 
	 } 
}

void GMYawControl_Out()
{
   ;
}

void Encoder_Err()
{
	;
}


