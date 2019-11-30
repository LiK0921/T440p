#include "mpu6050.h"
#include "parameter.h"

u8 mpu6050_buf[30];
float mpu6050_tmp[ITEMS];

float Roll,Pitch,Yaw;    				//姿态角

void MPU6050_Get(u8 data)
{
	static u8 mpu_state = 0,data_cnt=0;
	switch(mpu_state)
	{
		case 0: 
			if(data == 0x55)
				mpu_state = 1;
		break;
		case 1: 
			if(data == 0x51)
				mpu_state = 2;
			else if(data == 0x52)
				mpu_state = 3;
			else if(data == 0x53)
				mpu_state = 4;
		break;
		case 2:
			if(data_cnt < 8)
			  mpu6050_buf[data_cnt++] = data;
			else
			{
			  mpu_state = 0;
				mpu6050_tmp[A_X] = ((short)(mpu6050_buf[1]<<8 | mpu6050_buf[0]))/32768.0f*16;      //X轴加速度
				mpu6050_tmp[A_X] = ((short)(mpu6050_buf[3]<<8 | mpu6050_buf[2]))/32768.0f*16;      //Y轴加速度
				mpu6050_tmp[A_X] = ((short)(mpu6050_buf[5]<<8 | mpu6050_buf[4]))/32768.0f*16;      //Z轴加速度
				mpu6050_tmp[TEM] = ((short)(mpu6050_buf[7]<<8 | mpu6050_buf[6]))/340.0f+36.25f;    //温度
			}
		break;
		case 3:
			if(data_cnt < 16)
				mpu6050_buf[data_cnt++] = data;
	 	  else
			{
				mpu_state = 0;
				mpu6050_tmp[G_X] = ((short)(mpu6050_buf[9]<<8 | mpu6050_buf[8]))/32768.0f*2000;      //X轴角速度
				mpu6050_tmp[G_Y] = ((short)(mpu6050_buf[11]<<8 | mpu6050_buf[10]))/32768.0f*2000;      //Y轴角速度
				mpu6050_tmp[G_Z] = ((short)(mpu6050_buf[13]<<8 | mpu6050_buf[12]))/32768.0f*2000;      //Y轴角速度
				mpu6050_tmp[TEM] = ((short)(mpu6050_buf[15]<<8 | mpu6050_buf[14]))/340.0f+36.25f;      //温度
			}
		break;  
		case 4:
			if(data_cnt < 24)
				mpu6050_buf[data_cnt++] = data;
	    else 
			{
				mpu_state = 0;
				Roll  = ((short)(mpu6050_buf[17]<<8 | mpu6050_buf[16]))/32768.0*180;      //X轴加速度
				Pitch = ((short)(mpu6050_buf[19]<<8 | mpu6050_buf[18]))/32768.0*180;      //Y轴加速度
				Yaw   = ((short)(mpu6050_buf[21]<<8 | mpu6050_buf[20]))/32768.0*180;      //Z轴加速度
				mpu6050_tmp[TEM] = ((short)(mpu6050_buf[23]<<8 | mpu6050_buf[22]))/340.0+36.25;      //温度
				data_cnt = 0;
			}
		break; 
		default:  break;
	}
}


