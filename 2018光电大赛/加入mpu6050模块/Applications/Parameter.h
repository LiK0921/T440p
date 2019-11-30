#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "stm32f4xx.h"

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
		PID4,
		PID5,
		PID6,

		PIDITEMS
};

typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;

}pid_t;

typedef struct 
{
  float x;
	float y;
	float z;
}xyz_f_t;

typedef union
{
	uint8_t raw_data[64];
	struct
	{
		xyz_f_t Accel;
		xyz_f_t Gyro;
		xyz_f_t Mag;
		xyz_f_t vec_3d_cali;
		uint32_t mpu_flag;
		float Acc_Temperature;
		float Gyro_Temperature;
	}Offset;
}sensor_setup_t;

typedef union
{
 uint8_t raw_data[193];
 struct
 {
	 u8 first_init;
	 pid_t pid_1;
	 pid_t pid_2;
	 pid_t pid_3;
	 pid_t pid_4;
	 pid_t pid_5;
	 pid_t pid_6;
	 pid_t pid_7;
	 pid_t pid_8;
	 pid_t pid_9;
	 pid_t pid_10;
	 pid_t pid_11;
	 pid_t pid_12;
	 pid_t pid_13;
	 pid_t pid_14;
	 pid_t pid_15;
	 pid_t pid_16;
	 pid_t pid_17;
	 pid_t pid_18;
 	  
 }groups;

}pid_setup_t;

extern pid_setup_t pid_setup;

void Para_ResetToFactorySetup(void);

void PID_Para_Init(void);


#endif



