#ifndef __CONTROL_H
#define __CONTROL_H

#include "system.h"

#define PITCH 0
#define YAW	  1

#define M 0
#define G 1
#define A 2
#define S 3

#define POSITION 0
#define SPEED    1
#define SUM      2

#define Bomb_Angle 112347

#ifdef Infantry_Emerge						

#define Y_Mid   	4790  
#define Y_Right	 	2810  
#define Y_Left  	6930

#define P_Mid  		4330			
#define P_High  	4913
#define P_Low   	3670
#endif


#ifdef Infantry_OpenFire

#define Y_Mid   	4290  
#define Y_Right	 	2270   
#define Y_Left  	6330

#define P_Mid   	4145			
#define P_High 		4560
#define P_Low  	 	3490
#endif


#ifdef Infantry_Yixiuge2						

#define Y_Mid   	4885  
#define Y_Right	 	2900   
#define Y_Left  	7000

#define P_Mid  		3090			
#define P_High  	3600
#define P_Low   	2415
#endif


#ifdef Infantry_Carrot

#define Y_Mid   3100  
#define Y_Right 1200  
#define Y_Left  5000

#define P_Mid   5510			
#define P_High  5985
#define P_Low   4900
#endif


//云台
extern int16_t Moto_angle[2];
extern float Yaw_Target[2],Pitch_Target[2],Gimbal_PIDTerm[2];			

//底盘			
extern float Chassis_PIDTerm[4],Speed_Z;

//送弹电机
extern float S_Target[3],S_PIDTerm;
void RM2006_PID_Position_Control(void);

//陀螺仪掉线保护
extern bool sensor_online; //到时放在串口5

//失控保护
enum system_monitor
{	
	Error_Mode=0,
	Normal_Mode
};

extern int System_Mode;
enum system_mode
{
	None_Mode       =0,
	Rc_Machine_Mode =1,
	Rc_Gyro_Mode    =2,
	Key_Machine_Mode=3,
	Key_Gyro_Mode   =4,
	BigBuff_Mode    =5,
	SmallBuff_Mode  =6,
	ManualBuff_Mode =7
};
#endif

