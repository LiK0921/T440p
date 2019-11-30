#include "filter.h"

#define ACC_CUT_OFF_FREQUENCY 10.0f		//陀螺仪低通滤波器截止频率50Hz
#define GYRO_LPF_CUT 60.0f						//陀螺仪低通滤波器截止频率30Hz
#define PI 3.141592653f

#define ACC_1G 			4096		//由加速度计的量程确定
#define ACC_LPF_CUT 50.0f		//加速度低通滤波器截止频率50Hz

#define GYRO_LPF_CUT 60.0f		//陀螺仪低通滤波器截止频率30Hz
#define GYRO_CF_TAU 2.5f


float LPF_1st_factor[2];
float CF_factor;
float imulooptime=0.02;
float deltaT,tau;

LPF_2st	LPF_2St_factor;


/*----------------------一阶低通滤波器系数计算-------------------------*/
void LPF_1st_Factor_Cal(void)
{
	LPF_1st_factor[0] = imulooptime / (imulooptime + 1 / (2 * PI * ACC_CUT_OFF_FREQUENCY));	//加速度滤波
	LPF_1st_factor[1] = imulooptime / (imulooptime + 1 / (2 * PI * GYRO_LPF_CUT));		//陀螺仪滤波
}
/*----------------------一阶低通滤波器------------------------*/
void LPF_1st(int16_t*oldData ,int16_t* newData,float factor,int num)		//acc--0,gyro--1;
{
	int i;
	for(i=0;i<num;i++)
	{
		oldData[i] = oldData[i] * (1 - factor) + newData[i] * factor;
	}
}
/*----------------------二阶低通滤波器系数计算-------------------------*/

void LPF_2nd_Factor_Cal(float deltaT, float Fcut,int num)
{
	float a = 1 / (2 * PI * Fcut * deltaT);
	LPF_2St_factor.b0[num] = 1 / (a*a + 3*a + 1);
	LPF_2St_factor.a1[num] = (2*a*a + 3*a) / (a*a + 3*a + 1);
	LPF_2St_factor.a2[num] = (a*a) / (a*a + 3*a + 1);
}
/*----------------------二阶低通滤波器------------------------*/

void LPF_2nd(int16_t* lpf_2nd_data,int16_t* newData,int num)
{
	char i;
	float lpf_2nd_preout[3],lpf_2nd_lastout[3];
	for(i=0;i<3;i++)
	{
		lpf_2nd_data[i] = newData[i] * LPF_2St_factor.b0[num] + lpf_2nd_lastout[i] * LPF_2St_factor.a1[num] - lpf_2nd_preout[i] * LPF_2St_factor.a2[num];
		lpf_2nd_preout[i] = lpf_2nd_lastout[i];
		lpf_2nd_lastout[i] = lpf_2nd_data[i];
	}
}
/*----------------------互补滤波器系数计算-------------------------*/
void CF_Factor_Cal(void)
{
	CF_factor = tau / (deltaT + tau);
}
/*----------------------一阶互补滤波器-----------------------------*/
int CF_1st(int16_t* gyroData, int16_t* accData)
{ 	
	char i;
	int16_t temp[3];
	for(i=0;i<3;i++)
	{
	 temp[i] = (*(gyroData+i) * CF_factor + *(accData+i) *(1 - CF_factor));	
	}
	return *temp;
}
void Filter_Init(void)
{

	LPF_1st_Factor_Cal();
	LPF_2nd_Factor_Cal(imulooptime,GYRO_LPF_CUT,1);
//	CF_Factor_Cal();		//互补滤波


//	LPF_2nd_Factor_Cal(imulooptime,ACC_LPF_CUT,0);
//	LPF_2nd_Factor_Cal(imulooptime,GYRO_LPF_CUT,1);

}
/*----------------------滑动滤波器-----------------------------*/
#define FILTER_NUM 20
void Prepare_Data(int16_t *acc_in,int16_t *acc_out)
{
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = acc_in[0];
	ACC_Y_BUF[filter_cnt] = acc_in[1];
	ACC_Z_BUF[filter_cnt] = acc_in[2];
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	acc_out[0] = temp1 / FILTER_NUM;
	acc_out[1] = temp2 / FILTER_NUM;
	acc_out[2] = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}
