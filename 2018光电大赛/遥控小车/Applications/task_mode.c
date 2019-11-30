#include "task_mode.h"
#include "rc.h"
#include "init.h"
#include "CanBusTask.h"
#include "ControlTask.h"

u8 mode_value[10];
u8 mode_state,mode_state_old;//分别记录当前状态和历史状态
u8 task_on,state_prepare;
void mode_check(float *ch_in,u8 *mode_value)
{ 
  if(*(ch_in + AUX2) >200)
		task_on = 1;
  else
	{		
		task_on = 0;
		state_prepare = 0;
		Set_CM_Speed(CAN1,0,0,0,0);
	}
  
	if(*(ch_in+AUX1) <-200) //AUX1表示遥控器上方最右边的三个档位
	{
		mode_state = 0;//遥控控制;
		state_prepare = 0;
	}
	else if(*(ch_in+AUX1) >200)  //紧急停机模式
	{
		mode_state = 2;//
		state_prepare = 0;
	}
	else
	{
		mode_state = 1;//云台电机角度控制
		if(state_prepare == 0)
		{
			GMPPosition_to_record = CM2Encoder.ecd_angle;
			state_prepare = 1;
		}
	}
 
	//===========   ===========
	mode_state_old = mode_state; //历史模式
}

