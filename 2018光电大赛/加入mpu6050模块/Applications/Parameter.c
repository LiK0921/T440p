#include "Parameter.h"
#include "string.h"
#include "ff.h"
#include "ControlTask.h"
#include "include.h"

u8 flash_init_error;
sensor_setup_t sensor_setup;
pid_setup_t pid_setup;

void Para_ResetToFactorySetup(void)
{
	pid_setup.groups.pid_1.kp = 1.0f;
	pid_setup.groups.pid_1.ki = 1.0f;
	pid_setup.groups.pid_1.kd = 1.0f;
	
	pid_setup.groups.pid_2.kp = 1.0f;
	pid_setup.groups.pid_2.ki = 1.0f;	
	pid_setup.groups.pid_2.kd = 1.0f;	
	
	
	pid_setup.groups.pid_3.kp = 1.0f;
	pid_setup.groups.pid_3.ki = 1.0f;	
	pid_setup.groups.pid_3.kd = 1.0f;	
	
	pid_setup.groups.pid_4.kp = 1.0f;
	pid_setup.groups.pid_4.ki = 1.0f;	
	pid_setup.groups.pid_4.kd = 1.0f;	
	
	pid_setup.groups.pid_5.kp = 1.0f;
	pid_setup.groups.pid_5.ki = 1.0f;	
	pid_setup.groups.pid_5.kd = 1.0f;	
	
	pid_setup.groups.pid_6.kp = 1.0f;
	pid_setup.groups.pid_6.ki = 1.0f;	
	pid_setup.groups.pid_6.kd = 1.0f;	
	
	pid_setup.groups.pid_7.kp = 1.0f;
	pid_setup.groups.pid_7.ki = 1.0f;	
	pid_setup.groups.pid_7.kd = 1.0f;	
	
	pid_setup.groups.pid_8.kp = 1.0f;
	pid_setup.groups.pid_8.ki = 1.0f;	
	pid_setup.groups.pid_8.kd = 1.0f;	
	
	pid_setup.groups.pid_9.kp = 1.0f;
	pid_setup.groups.pid_9.ki = 1.0f;	
	pid_setup.groups.pid_9.kd = 1.0f;	
	
	pid_setup.groups.pid_9.kp = 1.0f;
	pid_setup.groups.pid_9.ki = 1.0f;
	pid_setup.groups.pid_9.kd = 1.0f;
	
	pid_setup.groups.pid_10.kp = 1.0f;
	pid_setup.groups.pid_10.ki = 1.0f;
	pid_setup.groups.pid_10.kd = 1.0;
	
	pid_setup.groups.pid_11.kp = 1.0f;
	pid_setup.groups.pid_11.ki = 1.0f;
	pid_setup.groups.pid_11.kd = 1.0f;
	
	pid_setup.groups.pid_12.kp = 1.0f;
	pid_setup.groups.pid_12.ki = 1.0f;
	pid_setup.groups.pid_12.kd = 1.0;
	
	pid_setup.groups.pid_13.kp = 1.0f;
	pid_setup.groups.pid_13.ki = 1.0f;
	pid_setup.groups.pid_13.kd = 1.0f;
	
	pid_setup.groups.pid_14.kp = 1.0f;
	pid_setup.groups.pid_14.ki = 1.0f;
	pid_setup.groups.pid_14.kd = 1.0f;
	
	pid_setup.groups.pid_15.kp = 1.0f;
	pid_setup.groups.pid_15.ki = 1.0f;
	pid_setup.groups.pid_15.kd = 1.0f;
	
	pid_setup.groups.pid_16.kp = 1.0f;
	pid_setup.groups.pid_16.ki = 1.0f;
	pid_setup.groups.pid_16.kd = 1.0;
	
	pid_setup.groups.pid_17.kp = 1.0f;
	pid_setup.groups.pid_17.ki = 1.0f;
	pid_setup.groups.pid_17.kd = 1.0f;
	
	pid_setup.groups.pid_18.kp = 1.0f;
	pid_setup.groups.pid_18.ki = 1.0f;
	pid_setup.groups.pid_18.kd = 1.0f;
}

void PID_Para_Init()
{
	GMYawControl_ParaInit();
}



