#ifndef __FILTER_H
#define	__FILTER_H

#include "system.h"

extern float imulooptime;
extern float deltaT,tau;
extern float LPF_1st_factor[2];
typedef struct 
{   
	float a1[2];
	float a2[2];
	float	b0[2];
} LPF_2st;

void Filter_Init(void);
void LPF_1st(int16_t* oldData,int16_t* newData,float factor,int num);
void LPF_2nd(int16_t* lpf_2nd_data,int16_t* newData,int num);
int CF_1st(int16_t* gyroData, int16_t* accData);
void Prepare_Data(int16_t *acc_in,int16_t *acc_out);

#endif
