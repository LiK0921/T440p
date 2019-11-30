#include "schedule.h"
#include "timer.h"
#include "data_transfer.h"

s16 loop_cnt;

loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //ÿ�ۼ�һ�Σ�֤��������Ԥ��������û�����ꡣ
	}
	else
	{	
		loop.check_flag = 1;	//�ñ�־λ��ѭ�����������
	}
	
}

float time_to_test;

void Duty_1ms()
{
  Get_Cycle_T(1)/1000000.0f;

	ANO_DT_Data_Exchange();
}

float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0)/1000000.0f; 						//��ȡ�ڻ�׼ȷ��ִ������
	
	test[0] = GetSysTime_us()/1000000.0f;
	
	
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2)/1000000.0f;								//��ȡ�⻷׼ȷ��ִ������
	
	test[2] = GetSysTime_us()/1000000.0f;

 	
	
	test[3] = GetSysTime_us()/1000000.0f;
}

void Duty_10ms()
{

}

void Duty_20ms()
{
	//Parameter_Save();
}

void Duty_50ms()
{
	//Mode();	
//	mode_check(CH_filter,mode_value);
}


void Duty_Loop()   					//�����������Ϊ1ms���ܵĴ���ִ��ʱ����ҪС��1ms��
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//����1ms������
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//����2ms������
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//����5ms������
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}


