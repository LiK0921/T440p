#include "control.h"

#define Emerge 		1
#define OpenFire  4
#define Yixiuge2 	5
#define Carrot  	10

#ifdef Infantry_Emerge

#define pitch_High				 264
#define pitch_Low					-314

#define Rocking_Right      4530   //Y_Right+1720 
#define Rocking_Left       5210		//Y_Left -1720

#define Open_Angle 				 75  
#define Close_Angle 			 30 
#define Fri_Power 				 307 
#endif


#ifdef Infantry_OpenFire

#define pitch_High				 292 
#define pitch_Low					-400

#define Rocking_Right      3990   
#define Rocking_Left       4610   


#define Open_Angle 				 75  
#define Close_Angle 			 30 
#define Fri_Power 				 307 
#endif


#ifdef Infantry_Yixiuge2 

#define pitch_High				 320
#define pitch_Low					-230

#define Rocking_Right      4620   
#define Rocking_Left       5280		

#define Fri_Power 				 308
#define Open_Angle 				 75
#define Close_Angle 			 30
#endif


#ifdef Infantry_Carrot 

#define pitch_High				 264
#define pitch_Low					-314

#define Rocking_Right      2950   
#define Rocking_Left       3250		

#define Fri_Power 				 405
#define Open_Angle 				 95
#define Close_Angle 			 50
#endif


#define Rocking_Limit 1000 
#define Speed_Max     10000
#define Offline_Speed 5000
#define High_Speed    3000

extern int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
extern int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
extern float constrain(float amt, float low, float high);
extern int constrain_int(int amt,int low,int high);

extern unsigned char Sudoku_Num;
extern int Infrared_Num,Record_Num,BombInit;
extern bool Protect_flag,Target_Static;
extern Measure_t Measure;
extern Judge_t Judge;

//�˶�ģʽ�л�
bool Rc_flag=0,Key_flag=1,First_Key=1,Distinction_flag;
int System_Mode,Mode_Record[2],GA_Record,Infantry_ID;

void Get_System_Mode()
{	
		if(RC_Ctl.rc.s2==RC_SW_UP)//s2������ ����ģʽ													 
		{
				Key_flag=1;
				Rc_flag=0;	
				if(First_Key)
				{
						System_Mode = Key_Gyro_Mode;
						First_Key=0;
				}
		}
		else if(RC_Ctl.rc.s2==RC_SW_MID)
		{
				System_Mode = Rc_Gyro_Mode;				
				First_Key=1;
				Key_flag=0;
				Rc_flag=1;
		}
		else if(RC_Ctl.rc.s2==RC_SW_DOWN)
		{
				System_Mode = Rc_Machine_Mode;
				First_Key=1;
				Key_flag=0;
				Rc_flag=1;
		}		
		
		if(Key_flag)   																								
		{	
				if((RC_Ctl.key.v & 0x10) == KEY_PRESSED_OFFSET_SHIFT||(RC_Ctl.key.v & 0x200) == KEY_PRESSED_OFFSET_F||(RC_Ctl.key.v & 0x400) == KEY_PRESSED_OFFSET_G)
				{
						if(System_Mode == Key_Machine_Mode)
						{
								Distinction_flag=1;
						}
						else if(System_Mode == Key_Gyro_Mode)
						{
								Distinction_flag=0;
						}		
				}
				if((RC_Ctl.key.v & 0x10) == KEY_PRESSED_OFFSET_SHIFT)//Shift�л������ģʽ
				{							
						System_Mode = BigBuff_Mode;
						Mode_Record[0]=0;//����G��ر�־λ
				}			
				else if((RC_Ctl.key.v & 0x200) == KEY_PRESSED_OFFSET_F)//F�л�С���ģʽ  
				{																		
						System_Mode = SmallBuff_Mode;
						Mode_Record[0]=0;//����G��ر�־λ
				}				
				else if(((RC_Ctl.key.v & 0x01) == KEY_PRESSED_OFFSET_W||(RC_Ctl.key.v & 0x02) == KEY_PRESSED_OFFSET_S||    //��WSAD������л�ԭ����ģʽ
								 (RC_Ctl.key.v & 0x04) == KEY_PRESSED_OFFSET_A||(RC_Ctl.key.v & 0x08) == KEY_PRESSED_OFFSET_D)&&
								 (System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode))
				{
						if(Distinction_flag)
						{
								System_Mode = Key_Machine_Mode;
						}
						else
						{														
								System_Mode = Key_Gyro_Mode;
						}
				}	
					
				if((RC_Ctl.key.v & 0x400) == KEY_PRESSED_OFFSET_G&&Mode_Record[0]==0)//��G�����ִ���ģʽ
				{
						Mode_Record[0]=1;				
						System_Mode = ManualBuff_Mode;
				}
				else if((RC_Ctl.key.v & 0x400) != KEY_PRESSED_OFFSET_G&&Mode_Record[0]==1)
				{
						Mode_Record[0]=2;
				}
				else if((RC_Ctl.key.v & 0x400) == KEY_PRESSED_OFFSET_G&&Mode_Record[0]==2)
				{
						Mode_Record[0]=3;
						if(Distinction_flag)
						{
								System_Mode = Key_Machine_Mode;
						}
						else
						{
								System_Mode = Key_Gyro_Mode;
						}
				}
				else if((RC_Ctl.key.v & 0x400) != KEY_PRESSED_OFFSET_G&&Mode_Record[0]==3)
				{
						Mode_Record[0]=0;
				}
				
				if(System_Mode == Key_Gyro_Mode||System_Mode == Key_Machine_Mode)	//ctrl�л���е
				{
						Distinction_flag=0;
						if(((RC_Ctl.key.v & 0x20) == KEY_PRESSED_OFFSET_CTRL)&&Mode_Record[1]==0)
						{
								Mode_Record[1]=1;
								System_Mode = Key_Machine_Mode;
						}
						else if(((RC_Ctl.key.v & 0x20) != KEY_PRESSED_OFFSET_CTRL)&&Mode_Record[1]==1)
						{
								Mode_Record[1]=2;
						}
						else if(((RC_Ctl.key.v & 0x20) == KEY_PRESSED_OFFSET_CTRL)&&Mode_Record[1]==2)
						{
								Mode_Record[1]=3;
								System_Mode = Key_Gyro_Mode;
						}
						else if(((RC_Ctl.key.v & 0x20) != KEY_PRESSED_OFFSET_CTRL)&&Mode_Record[1]==3)
						{
								Mode_Record[1]=0;									
						}
				}
		}
		if(System_Mode == None_Mode)
		{
				if(Rc_flag)
				{
						System_Mode = Rc_Machine_Mode;
				}
				else if(Key_flag)
				{
						System_Mode = Key_Machine_Mode;
				}
		}
}

//ϵͳ����ģʽ
bool Start_Mode=1;

void System_Start()
{
		#ifdef Infantry_Emerge
		Infantry_ID=Emerge;
		#endif
		#ifdef Infantry_OpenFire
		Infantry_ID=OpenFire;
		#endif
		#ifdef Infantry_Yixiuge2
		Infantry_ID=Yixiuge2;
		#endif
		#ifdef Infantry_Carrot
		Infantry_ID=Carrot;
		#endif
		if(Rc_flag)
		{
				System_Mode = Rc_Machine_Mode;
		}
		else if(Key_flag)
		{
				System_Mode = Key_Machine_Mode;	
		}		
		if(abs(-Pitch_Target[M] + Moto_angle[PITCH])<=10&&abs(Yaw_Target[M] - Moto_angle[YAW])<=10)//��еģʽ������̨
		{
				Start_Mode=0;	
				System_Mode = None_Mode;	
		}
}

//��̨
float  Yaw_Target[2]={Y_Mid,0},Pitch_Target[2]={P_Mid,0},Angle_Error[2],Rate_Target[2],Rate_Error[2],pitch_add=13;
float  PTerm[2],ITerm[2],DTerm[2],Gimbal_PIDTerm[2],Gimbal_Error[2];
float Zero_Compose[2],Zero_Num=0.5,Static_Yaw;
float Rc_Yaw=0.008,Mouse_Yaw[2]={2.5,8},RC_Pitch[2]={0.01,0.005},Mouse_Pitch[2]={3,9.5};
int pitch_dev=200;
bool Static_flag;

bool Go_Back_flag;
extern bool Roam_Shoot;
bool First_Roam=1;
void Turn_Back()
{	
		if((RC_Ctl.key.v & 0x2000) == KEY_PRESSED_OFFSET_C)   //��C��ͷ
		{
				Go_Back_flag=1;
		}
		if(Go_Back_flag&&((RC_Ctl.key.v & 0x2000) != KEY_PRESSED_OFFSET_C))    
		{
				Yaw_Target[G] -= 1800;
				Go_Back_flag=0;
		}
}

void Gimbal_Remote()//��̨����ָ��
{
		//yaw
		if((Moto_angle[YAW]<=(Y_Right+750)&&(RC_Ctl.mouse.x>0||RC_Ctl.rc.ch0>RC_CH_VALUE_OFFSET))	//������̨����̷���Ƕ�
		 ||(Moto_angle[YAW]>=(Y_Left -300)&&(RC_Ctl.mouse.x<0||RC_Ctl.rc.ch0<RC_CH_VALUE_OFFSET)))
		{
				RC_Ctl.mouse.x=0;
				RC_Ctl.rc.ch0=RC_CH_VALUE_OFFSET;
		}

		if(System_Mode == Key_Gyro_Mode&&!Roam_Shoot)//����������
		{
				Turn_Back();
				Yaw_Target[G] += -RC_Ctl.mouse.x/Mouse_Yaw[G];							
		}
		else if(System_Mode == Rc_Gyro_Mode)//ң����������
		{
				Yaw_Target[G] += (-RC_Ctl.rc.ch0 + RC_CH_VALUE_OFFSET ) * Rc_Yaw;				
		}	
		else if(((System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)&&Record_Num!=3)||Roam_Shoot)//���̻�е
		{
				if(Roam_Shoot&&First_Roam)
				{
//						Yaw_Target[M] += 400;
						First_Roam=0;
				}
				Yaw_Target[M] += -RC_Ctl.mouse.x / Mouse_Yaw[M];				 	
		}
		if(!Roam_Shoot)
				First_Roam=1;
		
		//pitch
		if(System_Mode == Key_Gyro_Mode||((System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)&&Record_Num!=3))//����������
		{					
				Pitch_Target[G] += RC_Ctl.mouse.y/Mouse_Pitch[G];
		}
		else if(System_Mode == Rc_Gyro_Mode)//ң���������� 
		{		
				Pitch_Target[G] += (-RC_Ctl.rc.ch1+RC_CH_VALUE_OFFSET ) * RC_Pitch[G];			
		}		
		else if(System_Mode == Key_Machine_Mode)//���̻�е
		{		
				Pitch_Target[M] += RC_Ctl.mouse.y/Mouse_Pitch[M];
		}
		else if(System_Mode == Rc_Machine_Mode||System_Mode == Rc_Gyro_Mode)//ң������е
		{	
				Pitch_Target[M] += (-RC_Ctl.rc.ch1 + RC_CH_VALUE_OFFSET ) * RC_Pitch[M];
//				Pitch_Target[M] = P_Mid + (-RC_Ctl.rc.ch1 + RC_CH_VALUE_OFFSET ) * 1.5f;
		} 
}

void Gimbal_Yaw_Gyro_PID_Outter_Control(void)//yaw��������ģʽ�⻷
{		
		Yaw_Target[M] = Y_Mid;
		if(Yaw_Target[G]>1799)
		{
				Yaw_Target[G] = abs(Yaw_Target[G]) - 3598;
		}
		else if(Yaw_Target[G]<-1799)
		{
				Yaw_Target[G] = 3598 - abs(Yaw_Target[G]);
		}
		Yaw_Target[G] = constrain(Yaw_Target[G],-1799,1799);
		if(abs(Zero_Compose[YAW])<=10&&!Static_flag&&Target_Static)//������yaw��Ư�ƴ���
		{							
				Static_flag=1;
				Static_Yaw=yaw;
		}	
		else if(abs(Zero_Compose[YAW])>=35)
		{
				Static_flag=0;
		}
		if(Static_flag)
		{							                          
				Angle_Error[YAW] = (Yaw_Target[G] - Static_Yaw * 10)/5;
				Zero_Compose[YAW]=0;
		}
		else 
		{
				Angle_Error[YAW] = Yaw_Target[G] - yaw * 10; 	
		}	
		if(Angle_Error[YAW]<=-1799)//������yaw���ٽ紦��
		{
				Angle_Error[YAW] = 3598 - abs(Angle_Error[YAW]);				
		}
		if(Angle_Error[YAW]>=1799)
		{
				Angle_Error[YAW]= abs(Angle_Error[YAW]) - 3598;
		}	
		Angle_Error[YAW] = constrain(Angle_Error[YAW],-1799,1799);
		Rate_Target[YAW] = Angle_Error[YAW] * PID.Gimbal.Level[YAW][G]	;   			//��ȡ���ٶ�Ŀ��  
		Rate_Target[YAW] = constrain(Rate_Target[YAW],-5000,5000); 
		Rate_Error[YAW] = Rate_Target[YAW] - Zero_Compose[YAW];										//��ȡ���ٶ����
		Rate_Error[YAW] = constrain(Rate_Error[YAW],-5000,5000);
}

void Gimbal_Yaw_Machine_PID_Outter_Control(void)                            	//��еģʽ�⻷
{		
		Yaw_Target[G] = yaw * 10;
		Yaw_Target[M] = constrain(Yaw_Target[M],Y_Right,Y_Left);	
		Angle_Error[YAW] = Yaw_Target[M] - Moto_angle[YAW]; 												
		if(Start_Mode)
		{
				Rate_Target[YAW] = constrain(Angle_Error[YAW] * PID.Gimbal.Level[YAW][M],-2000,2000);
		}
		else
		{
				if(System_Mode == Key_Machine_Mode||System_Mode == Rc_Machine_Mode)
				{
						Rate_Target[YAW] = constrain(Angle_Error[YAW] * PID.Gimbal.Level[YAW][M],-5000,5000);
				}
				else
						Rate_Target[YAW] = constrain(Angle_Error[YAW] * PID.Gimbal.Level[YAW][A],-5000,5000);
		}			
		Rate_Error[YAW] = Rate_Target[YAW] - Zero_Compose[YAW];						
		Rate_Error[YAW] = constrain(Rate_Error[YAW],-5000,5000);    								   
}

void Gimbal_Yaw_PID_Inner_Control(void)                             					 //Yaw���ڻ�
{
		float P,I;
		if((System_Mode == Key_Gyro_Mode||System_Mode == Rc_Gyro_Mode)&&!Roam_Shoot)  				
		{	
					P=PID.Gimbal.P[YAW][G];
					I=PID.Gimbal.I[YAW][G];
		}
		else if(System_Mode == Key_Machine_Mode||System_Mode == Rc_Machine_Mode||Roam_Shoot)
		{
					P=PID.Gimbal.P[YAW][M];
					I=PID.Gimbal.I[YAW][M];
		}
		else if(System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)
		{
					P=PID.Gimbal.P[YAW][A];
					I=PID.Gimbal.I[YAW][A];
		}
		else
		{
					P=0;
					I=0;
		}

		Gimbal_Error[YAW] = Rate_Error[YAW];
		PTerm[YAW] = P * Gimbal_Error[YAW];
	
		if(Static_flag==1)
		{
				I=0;
		}
		ITerm[YAW] += I * Gimbal_Error[YAW] *  PID.Looptime;
	  ITerm[YAW] = constrain(ITerm[YAW],- PID.Gimbal.IMax,+ PID.Gimbal.IMax);

		Gimbal_PIDTerm[YAW] = -constrain(PTerm[YAW]+ITerm[YAW],-5000,5000);
}

void Gimbal_Pitch_Gyro_PID_Outter_Control(void)                       //pitch��������ģʽ�⻷    
{
		Pitch_Target[M] = Moto_angle[PITCH];
		if(Moto_angle[YAW]>=Y_Right+800&&Moto_angle[YAW]<=Y_Left-800)
		{
				Pitch_Target[G] = constrain(Pitch_Target[G],pitch_Low,pitch_High);
		}
		else
				Pitch_Target[G] = constrain(Pitch_Target[G],pitch_Low,pitch_High-100);//��ֹײ������װ��
		Angle_Error[PITCH] = constrain( Pitch_Target[G] - pitch * 10,-500,500);
//		PID.Gimbal.Level[PITCH][A] = abs(-0.005 * abs(Angle_Error[PITCH]) + pitch_add);   
		PID.Gimbal.Level[PITCH][G] = abs(Angle_Error[PITCH] / 100) + 15;
		PID.Gimbal.Level[PITCH][A] = abs(Angle_Error[PITCH] / 200) + 13;
		if(System_Mode == Key_Gyro_Mode||System_Mode == Rc_Gyro_Mode)
		{
				Rate_Target[PITCH] = constrain(Angle_Error[PITCH] * PID.Gimbal.Level[PITCH][G],-5000,5000);
		}
		else
				Rate_Target[PITCH] = constrain(Angle_Error[PITCH] * PID.Gimbal.Level[PITCH][A],-5000,5000);											
		Rate_Error[PITCH] = -constrain(Rate_Target[PITCH] - Zero_Compose[PITCH],-5000,5000);
}

void Gimbal_Pitch_Machine_PID_Outter_Control(void) //pitch���еģʽ�⻷        
{			
		Pitch_Target[G] = pitch * 10;
		if(Moto_angle[YAW]>=Y_Right+800&&Moto_angle[YAW]<=Y_Left-800)
		{
				Pitch_Target[M] = constrain(Pitch_Target[M],P_Low,P_High);
		}
		else
				Pitch_Target[M] = constrain(Pitch_Target[M],P_Low,P_High-200);//��ֹײ������װ��
		Angle_Error[PITCH] = -Pitch_Target[M] +  Moto_angle[PITCH]; 
		PID.Gimbal.Level[PITCH][M] = abs(Angle_Error[PITCH] / 500) + 10;
		PID.Gimbal.Level[PITCH][A] = abs(Angle_Error[PITCH] / pitch_dev) + pitch_add;
		if(Start_Mode)
		{
				Rate_Target[PITCH] = constrain(Angle_Error[PITCH] * PID.Gimbal.Level[PITCH][M],-2000,2000);
		}
		else
		{
				if(System_Mode == Key_Machine_Mode||System_Mode == Rc_Machine_Mode)
				{
						Rate_Target[PITCH] = constrain(Angle_Error[PITCH] * PID.Gimbal.Level[PITCH][M],-5000,5000);
				}
				else
						Rate_Target[PITCH] = constrain(Angle_Error[PITCH] * PID.Gimbal.Level[PITCH][A],-5000,5000);		
		}
		Rate_Error[PITCH] = constrain(Rate_Target[PITCH] + Zero_Compose[PITCH], -5000, 5000);    									 
}

void Gimbal_Pitch_PID_Inner_Control(void)                             
{
		float P,I;
		if(System_Mode == Key_Gyro_Mode||System_Mode == Rc_Gyro_Mode)  				
		{	
				P=PID.Gimbal.P[PITCH][G];
				I=PID.Gimbal.I[PITCH][G];
		}
		else if(System_Mode == Key_Machine_Mode||System_Mode == Rc_Machine_Mode)
		{
				P=PID.Gimbal.P[PITCH][M];
				I=PID.Gimbal.I[PITCH][M];
		}
		else if(System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)
		{
				P=PID.Gimbal.P[PITCH][A];
				I=PID.Gimbal.I[PITCH][A];
		}
		else
		{
				P=0;
				I=0;
		}
		
		Gimbal_Error[PITCH] = Rate_Error[PITCH];
		PTerm[PITCH] = P * Gimbal_Error[PITCH];
	
		ITerm[PITCH] += I * Gimbal_Error[PITCH] *  PID.Looptime;
	  ITerm[PITCH] = constrain(ITerm[PITCH],-PID.Gimbal.IMax,+PID.Gimbal.IMax);

		Gimbal_PIDTerm[PITCH] = constrain(PTerm[PITCH]+ITerm[PITCH],-5000,5000);
		if(Infantry_ID==Emerge)//Emerge��pitch����̨ �����ֵ �����������ʱ Ϊ��ʱ����ת
		{
				Gimbal_PIDTerm[PITCH] = -Gimbal_PIDTerm[PITCH];
		}
}

//����
bool Rocking_Math_flag=1;
int Rocking_Num=0,Rocking_Keep_flag;
float Rocking_Target=Rocking_Left,Rocking_Slope=3;

void Rocking_Control()//ҡ��ģʽ
{		
		Speed_Z = Rocking_Num;
		if(Rocking_Math_flag&&Rocking_Keep_flag!=1)
		{
				Rocking_Num -= Rocking_Slope;  
		}
		else if(!Rocking_Math_flag&&Rocking_Keep_flag!=2)
		{
				Rocking_Num += Rocking_Slope;
		}
		if((Moto_angle[YAW]>=Rocking_Left&&Rocking_Num>0)||(Moto_angle[YAW]<=Rocking_Right&&Rocking_Num<0)) //��ֹ����ҡ�ڽǶȹ��� Ҫע����ܴ���û���޶��Ƕ� �ٶ��Ѿ�Ϊ0�����
		{
				Rocking_Num=0;
		}			
		if(Rocking_Num <= -Rocking_Limit && Moto_angle[YAW]>Y_Mid) 			  //�ҵ��� ����û���м� �ٶ�������� ʹ֮����ֱ�������м� Ҫע����ܴ��ڵ����м� �ٶȻ�û���������
		{
				Rocking_Num = -Rocking_Limit;
				Rocking_Keep_flag=1;
		}
		else if(Rocking_Num >= Rocking_Limit && Moto_angle[YAW]<Y_Mid)		//����
		{
				Rocking_Num = Rocking_Limit;
				Rocking_Keep_flag=2;
		}
		if(Rocking_Num<0&&Moto_angle[YAW]<=Y_Mid)  		 										//�ҵ��� �����м�ֵ ��ʼ����
		{
				Rocking_Keep_flag=0;
				Rocking_Math_flag=0;
		}
		else if(Rocking_Num>0&&Moto_angle[YAW]>=Y_Mid) 										//����
		{
				Rocking_Keep_flag=0;
				Rocking_Math_flag=1;
		}
}

float Speed_Num[2],accmulate_ws_num=4,accmulate_ad_num=4,accmulate_d_num=20;
float Speed_X=0,Speed_Y=0,Speed_Z=0,Motor_Speed[4]; 
float Power_Num=10,Current_P=1; 
float Chassis_PIDTerm[4],Chassis_PTerm[4],Chassis_ITerm[4],Chassis_Max=9000;

int16_t Angle_Calculate;
bool Climb_flag;
extern bool Arsenal_Open_flag;
void Chassis_State()//������̬�ж�
{		
		Angle_Calculate = pitch * 22.8f + P_Mid;
		if(RC_Ctl.mouse.press_r==1)//�ֶ���������ģʽ
		{
				Climb_flag=1;
		}	
		if(Climb_flag||Moto_angle[PITCH] - Angle_Calculate>=350)//����ģʽ �ӻ��ٶ�б�� �������ϵ�� ��������ٶ�
		{
				Power_Num=6;
				PID.Speed.P=5;			
				accmulate_ws_num=0.8;			
				Chassis_Max=14000;
		}
		else if(Arsenal_Open_flag)//����ģʽ
		{
				Power_Num=7;	
				accmulate_ws_num=2;				
				Chassis_Max=8000;
		}
		else
		{			
				Power_Num=10;
				PID.Speed.P=2.5;
				accmulate_ws_num=4;
				Chassis_Max=9000;
		}
}

float Power_Limit=75;
void Power_Limit_Current_Control()	//�޵���
{
		realPower = (float)Measure.Power/100;
		if(Judge.Joule<=40)
		{
				if(abs(Motor_chassis[0][1])>High_Speed||abs(Motor_chassis[1][1])>High_Speed||abs(Motor_chassis[2][1])>High_Speed||abs(Motor_chassis[3][1])>High_Speed)//���Ӵﵽ��ת��ʱ�������
				{
						if(Infantry_ID == Yixiuge2)
						{
								Power_Limit=65;
						}
						else
						{
								Power_Limit=75;
						}
						if(realPower>=75)
						{
								if(Current_P>=0.6f)
								{
										Current_P -= 0.02f;
								}
								else
										Current_P -= 0.0075f;
						}
						else
								Current_P += 0.0001f;
				}
				else
				{
						Current_P -= 0.005f;
				}
				if(Judge.Joule<=10)
				{
						Current_P = (float)Judge.Joule/120;
				}
		}
		else
		{
				if(Judge.Joule>=50)
				{
						Current_P += 0.0001f;
				}
		}
		Current_P = constrain(Current_P,0,1);
}

void Chassis_PID_Control(int num,float Speed_Measure,float Speed_Target)
{
		static float  Error_Sum[4];	
		int16_t  Chassis_Error[4];	
		float P,I;
		if((RC_Ctl.rc.s2==RC_SW_UP)||(RC_Ctl.rc.s2==RC_SW_MID)||(RC_Ctl.rc.s2==RC_SW_DOWN))
		{
				P=PID.Speed.P;
				I=PID.Speed.I;
		}
		else
		{
				P=0;
				I=0;
		}
		//P
		Chassis_Error[num] = Speed_Measure-Speed_Target;
		Chassis_PTerm[num] = P * Chassis_Error[num];  
		
		//I                                          
		Error_Sum[num] += Chassis_Error[num]*0.5;	//������		
		Chassis_ITerm[num] = I * Error_Sum[num]* PID.Looptime;
		Chassis_ITerm[num] = constrain(Chassis_ITerm[num] ,-PID.Speed.IMax,+PID.Speed.IMax);
		
		if((Speed_Y==0&&Speed_X==0)||abs(Speed_Z)<=200)
		{
				Error_Sum[num]=0;
		}
		Chassis_PIDTerm[num] = -constrain((Chassis_PTerm[num] + Chassis_ITerm[num]) * Current_P,-Chassis_Max,+Chassis_Max);
}

bool Rocking_flag,Roam_Shoot;
void Chassis_Control()       //���̿���
{	
		Chassis_State();//������̬�ж�
	
		//��еģʽ Speed_Z
		if(System_Mode == Key_Machine_Mode)			//���̲���
		{
				Speed_Z = RC_Ctl.mouse.x * 25;
		}
		else if(System_Mode == Rc_Machine_Mode) //ң��������
		{
				Speed_Z = (RC_Ctl.rc.ch0-RC_CH_VALUE_OFFSET)*0.8;	
		}	
		
		//������ģʽ Speed_Z
		if(System_Mode == Key_Gyro_Mode||System_Mode == Rc_Gyro_Mode)
		{
				if((RC_Ctl.key.v & 0x80) == KEY_PRESSED_OFFSET_E)//E ��̨���̷���
				{
						Roam_Shoot=1;
						Rocking_flag=0;
				}
				else if((RC_Ctl.key.v & 0x40) == KEY_PRESSED_OFFSET_Q)
				{
						Rocking_flag=1;	
						Roam_Shoot=0;
				}
				if(Roam_Shoot)
				{
						Speed_Z-=0.5f;
				}
				else
				{
						Speed_Z = (Y_Mid-Moto_angle[YAW])*0.9f;			
				}
				if(Rocking_flag)
				{
						Rocking_Control();
						PID.Speed.P=2;
				}		
				else
				{
						Rocking_Slope=3;
						Rocking_Num=0;
						Rocking_Keep_flag=0;
						Rocking_Math_flag=1;
				}			
		}				
		
		if(Measure.Num>20)
		{
				Speed_Z = constrain(Speed_Z,RC_MIN*0.8f,RC_MAX*0.8f);
		}
		else if(Roam_Shoot)
		{
				Speed_Z = constrain(Speed_Z,RC_MIN/3,RC_MAX/3);
		}
		else if(Go_Back_flag)
		{				
				Speed_Z = constrain(Speed_Z,RC_MIN*3,RC_MAX*3);
		}	
		else
				Speed_Z = constrain(Speed_Z,RC_MIN,RC_MAX);
		
		if(Key_flag)//���̲���
		{
				if(Roam_Shoot)
				{
						Speed_Num[0]+=1;
						Speed_Num[1]+=2;
						Speed_Num[0] = constrain(Speed_Num[0],RC_MIN,RC_MAX);
						Speed_Num[1] = constrain(Speed_Num[1],RC_MIN*1.5f,RC_MAX*1.5f);
				}
				else
				{
						if((RC_Ctl.key.v & 0x01) == KEY_PRESSED_OFFSET_W)    
						{			
								Speed_Num[0] += accmulate_ws_num;								
						}
						else if((RC_Ctl.key.v & 0x02) == KEY_PRESSED_OFFSET_S)
						{
								Speed_Num[0] -= accmulate_ws_num;
						}
						else
						{
								if(Speed_Num[0]>0)
								{
										Speed_Num[0] -= accmulate_d_num;
										if(Speed_Num[0]<0)
										{
												Speed_Num[0]=0;
										}
								}
								else if(Speed_Num[0]<0)
								{
										Speed_Num[0] += accmulate_d_num;
										if(Speed_Num[0]>0)
										{
												Speed_Num[0]=0;
										}
								}
								else
								{
										Speed_Num[0] = 0;
								}					
						}
						if((RC_Ctl.key.v & 0x04) == KEY_PRESSED_OFFSET_A)
						{
								Speed_Num[1] -= accmulate_ad_num;
						}
						else if((RC_Ctl.key.v & 0x08) == KEY_PRESSED_OFFSET_D)
						{
								Speed_Num[1] += accmulate_ad_num;
						}				
						else 
						{
								if(Speed_Num[1]>0)
								{
										Speed_Num[1] -= accmulate_d_num;
										if(Speed_Num[1]<0)
										{
												Speed_Num[1]=0;
										}
								}
								else if(Speed_Num[1]<0)
								{
										Speed_Num[1] += accmulate_d_num;
										if(Speed_Num[1]>0)
										{
												Speed_Num[1]=0;
										}
								}
								else
								{
										Speed_Num[1] = 0;
								}
						}		
						Speed_Num[0] = constrain(Speed_Num[0],RC_MIN,RC_MAX);
						Speed_Num[1] = constrain(Speed_Num[1],RC_MIN,RC_MAX);
				}				
				Speed_Y = Speed_Num[0];
				Speed_X = Speed_Num[1];
		}
		else if(Rc_flag)  //ң��������
		{
				Speed_X = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
				Speed_Y = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
		}

		if(Rocking_flag||Roam_Shoot)
		{
				if((RC_Ctl.key.v & 0x04) == KEY_PRESSED_OFFSET_A||(RC_Ctl.key.v & 0x08) == KEY_PRESSED_OFFSET_D||
					 (RC_Ctl.key.v & 0x01) == KEY_PRESSED_OFFSET_W||(RC_Ctl.key.v & 0x02) == KEY_PRESSED_OFFSET_S||System_Mode != Key_Gyro_Mode)
				{			
						Roam_Shoot=0;
						Rocking_flag=0;
						Speed_X=0;
						Speed_Y=0;
						Speed_Z=0;
				}
		}
		
		Motor_Speed[0] = (+Speed_X + Speed_Y + Speed_Z) * Power_Num;		//0ǰ��	
		Motor_Speed[1] =-(-Speed_X + Speed_Y - Speed_Z) * Power_Num;		//1ǰ��
		Motor_Speed[2] = (-Speed_X + Speed_Y + Speed_Z) * Power_Num;		//2���
		Motor_Speed[3] =-(+Speed_X + Speed_Y - Speed_Z) * Power_Num;		//3�Һ�
		
		Motor_Speed[0] = constrain(Motor_Speed[0],-Speed_Max,+Speed_Max);
		Motor_Speed[1] = constrain(Motor_Speed[1],-Speed_Max,+Speed_Max);
		Motor_Speed[2] = constrain(Motor_Speed[2],-Speed_Max,+Speed_Max);
		Motor_Speed[3] = constrain(Motor_Speed[3],-Speed_Max,+Speed_Max);
		
		if(Measure.Update_flag)//���ݸ��²Ž��빦�ʻ�
		{
				Power_Limit_Current_Control();
				Measure.Update_flag=0;			
		}
		else if(Measure.Num>20)//���ʰ���� ǿ������
		{
				Current_P=1;
				accmulate_ws_num=2;
				accmulate_ad_num=2;
				Chassis_Max=6000;
				Motor_Speed[0] = constrain(Motor_Speed[0],-Offline_Speed,+Offline_Speed);
				Motor_Speed[1] = constrain(Motor_Speed[1],-Offline_Speed,+Offline_Speed);
				Motor_Speed[2] = constrain(Motor_Speed[2],-Offline_Speed,+Offline_Speed);
				Motor_Speed[3] = constrain(Motor_Speed[3],-Offline_Speed,+Offline_Speed);
		}
		Chassis_PID_Control(0,Motor_chassis[0][1],Motor_Speed[0]);
		Chassis_PID_Control(1,Motor_chassis[1][1],Motor_Speed[1]);
		Chassis_PID_Control(2,Motor_chassis[2][1],Motor_Speed[2]);
		Chassis_PID_Control(3,Motor_chassis[3][1],Motor_Speed[3]);
	
		if(System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)//���״̬ ���̾�ֹ	
		{
				Chassis_PIDTerm[0]=0;
				Chassis_PIDTerm[1]=0;
				Chassis_PIDTerm[2]=0;
				Chassis_PIDTerm[3]=0;
		}
}

//�������˶�ģʽ
void Gyro_Motion_Control()
{	
		Gimbal_Pitch_Gyro_PID_Outter_Control();
		if(!Roam_Shoot)
		{
				Gimbal_Yaw_Gyro_PID_Outter_Control();		
		}			
		else
				Gimbal_Yaw_Machine_PID_Outter_Control();
}

//��е�˶�ģʽ
void Machine_Motion_Control()
{	
		Yaw_Target[M] = Y_Mid;
		Gimbal_Pitch_Machine_PID_Outter_Control();						
		Gimbal_Yaw_Machine_PID_Outter_Control();
}

//���ģʽ
void Auto_Motion_Control()
{		
		Gimbal_Pitch_Gyro_PID_Outter_Control();
		Gimbal_Yaw_Machine_PID_Outter_Control();
}

void Gimbal_Control()
{
		switch(Infantry_ID)//���ٶȲ���
		{
				case Emerge:	Zero_Compose[YAW] = gyroz -  3;Zero_Compose[PITCH] = (gyrox + 40)*Zero_Num;break;
				case OpenFire:Zero_Compose[YAW] = gyroz + 13;Zero_Compose[PITCH] = (gyrox + 25)*Zero_Num;break;	
				case Yixiuge2:Zero_Compose[YAW] = gyroz + 15;Zero_Compose[PITCH] = (gyrox +  7)*Zero_Num;break;
				case Carrot:	Zero_Compose[YAW] = gyroz +  5;Zero_Compose[PITCH] = (gyrox +  5)*Zero_Num;break;
		}	
		if(System_Mode == Key_Gyro_Mode||System_Mode == Rc_Gyro_Mode)//������ģʽ
		{																					
				Gyro_Motion_Control();
		}	
		else if(System_Mode == Key_Machine_Mode||System_Mode == Rc_Machine_Mode)//��еģʽ
		{									
				Machine_Motion_Control();
		}	
		else if(System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)//���ģʽ
		{
				Auto_Motion_Control();
		}
		Gimbal_Pitch_PID_Inner_Control();
		Gimbal_Yaw_PID_Inner_Control();		
}

//Ħ����
bool Friction_ON,Fricition_Ready=0;
int Friction_Switch,Friction_Number=0;

void Friction_Control()     
{	
		if(Key_flag) //����������Ħ���� �ٵ�ת������ �Ҽ��ر�
		{
	 		  if(RC_Ctl.mouse.press_l==1&&Friction_Switch==0)
				{
						Friction_ON=1;
						Friction_Switch=1;
				}						
//				else if(RC_Ctl.mouse.press_l!=1&&Friction_Switch==1)  //������ʱ���˾Ͳ��ܹ���
//				{
//						Friction_Switch=2;
//				}
//				else if(RC_Ctl.mouse.press_r==1&&Friction_Switch==2)
//				{
//						Friction_ON=0;
//						Friction_Switch=3;
//						Fricition_Ready=0;
//				}	
//				else if((RC_Ctl.mouse.press_r!=1)&&Friction_Switch==3)
//				{
//						Friction_Switch=0;
//				}				
		}
		else if(Rc_flag)
		{
				if(RC_Ctl.rc.s1==RC_SW_UP&&Friction_Switch==0)
				{
						Friction_ON=1;
						Friction_Switch=1;
				}
				else if(RC_Ctl.rc.s1!=RC_SW_UP&&Friction_Switch==1)    
				{
						Friction_Switch=2;
				}
				else if((RC_Ctl.rc.s1==RC_SW_UP)&&Friction_Switch==2)
				{
						Friction_ON=0;
						Friction_Switch=3;	
						Fricition_Ready=0;
				}						
				else if(RC_Ctl.rc.s1!=RC_SW_UP&&Friction_Switch==3)
				{
						Friction_Switch=0;
				}	
		}
		if(Friction_ON)
		{		
				Friction_PWM((int16_t)Fri_Power,(int16_t)Fri_Power);
		}
		else 
		{
				Friction_PWM(0,0);
		}		
		
}	

//����
bool Arsenal_Open_flag,Infrared_Add,Infrared_Effect;
int Close_Num;

void Arsenal_Control()//��R��s1��������s2������ �����Ǻ�pitch��λ���������ͺ��� һ��ʱ����Զ��ϸǲ��رպ���
{
		if(System_Mode==Key_Gyro_Mode||System_Mode==Key_Machine_Mode||Rc_flag)
		{
				if(!Infrared_Effect)
				{
						if((RC_Ctl.key.v & 0x100) == KEY_PRESSED_OFFSET_R||(RC_Ctl.rc.s1==RC_SW_DOWN&&RC_Ctl.rc.s2==RC_SW_UP))
						{
								Infrared_Effect=1;
								Arsenal_Open_flag=1;
								Infrared_Add=1;//800ms�ڼ��60ms��һ��
								Close_Num=0;
						}	
				}
				if(Arsenal_Open_flag)
				{
						Pitch_Target[G] = 0;  //ʹpitchˮƽ
						Pitch_Target[M] = P_Mid;
						if(abs(Angle_Error[PITCH])<=3)
						{
								Servo_PWM((int16_t)Open_Angle);
						}
						if(Infrared_Effect)
						{			
								if(Infrared_Add)
								{
										Infrared_On();
										Infrared_Add=0;
								}
								else
								{
										Infrared_Num=0;
								}
						}
						else
								Infrared_Off();
						if(Close_Num>5) //6s�󵯲��Զ��ر�
						{
								Arsenal_Open_flag=0;		
						}
				}	
				else
				{
						Servo_PWM((int16_t)Close_Angle);
						Infrared_Off();
						Close_Num=0;	
						Infrared_Effect=0;
				}	
		}
}

//�͵����
bool Shoot_Effect,Key_Shoot=0,Start_Cir;
float S_Target[3]={0,3000,0},S_Error[2],S_PTerm[2],S_ITerm[2],S_DTerm[2],S_PIDTerm;

void RM2006_PID_Speed_Control()
{
		float P=0,I=0;
		if((RC_Ctl.rc.s2==RC_SW_UP)||(RC_Ctl.rc.s2==RC_SW_MID)||(RC_Ctl.rc.s2==RC_SW_DOWN))
		{
				P=PID.RM2006.P[SPEED];
				I=PID.RM2006.I[SPEED];	
		}
		else
		{
				P=0;
				I=0;
		}	
		//P
		S_Error[SPEED] = S_Target[SPEED] - RM06_Motor[1];
		S_PTerm[SPEED] = P * S_Error[SPEED];  		
		
		//I 
		S_ITerm[SPEED]+= I * S_Error[SPEED] * PID.Looptime;
		S_ITerm[SPEED] =constrain(S_ITerm[SPEED],-PID.RM2006.IMax[SPEED],+PID.RM2006.IMax[SPEED]);
		
		S_PIDTerm = constrain_int16(S_PTerm[SPEED]+S_ITerm[SPEED],0,4000);		
}

void RM2006_PID_Position_Control()
{
		static int32_t LastAngleError = 0;
		float P=0,I=0,D=0;		
		if((RC_Ctl.rc.s2==RC_SW_UP)||(RC_Ctl.rc.s2==RC_SW_MID)||(RC_Ctl.rc.s2==RC_SW_DOWN))
		{
				P=PID.RM2006.P[POSITION];
				I=PID.RM2006.I[POSITION];	
				D=PID.RM2006.D[POSITION];
		}
		else
		{
				P=0;
				I=0;
				D=0;
		}	
		
		if((S_Target[POSITION] < S_Target[SUM] )&&(S_Target[POSITION] - RM06_Angle.Total) <= 13870)//��ת��Ŀ��ֵ 
		{
				S_Target[POSITION] += 1387 ;	
				if(S_Target[POSITION] > S_Target[SUM])
						S_Target[POSITION] = S_Target[SUM];
		}
		else if((S_Target[POSITION] > S_Target[SUM] )&&(S_Target[POSITION]- RM06_Angle.Total) >= -13870)//��ת��Ŀ��ֵ
		{ 
				S_Target[POSITION] -= 1387;
				if(S_Target[POSITION] < S_Target[SUM])
						S_Target[POSITION] = S_Target[SUM];
		}
		//P
		S_Error[POSITION] = S_Target[POSITION] - RM06_Angle.Total;
		S_PTerm[POSITION] = P * S_Error[POSITION];  		
		//I 
		S_ITerm[POSITION] += I * S_Error[POSITION] * PID.Looptime;
		S_ITerm[POSITION] =constrain(S_ITerm[POSITION],-PID.RM2006.IMax[POSITION],+PID.RM2006.IMax[POSITION]);	
		//D
		S_DTerm[POSITION] = (S_Error[POSITION] - LastAngleError)/PID.Looptime*D;
			
		S_PIDTerm = constrain_int32(S_PTerm[POSITION] + S_ITerm[POSITION] + S_DTerm[POSITION],-8000,8000);	
		LastAngleError = S_Error[POSITION];
}


void Shoot_Control()   
{
		if(Key_flag)//���̲��� λ�û� 
		{	
				if(!Shoot_Effect)
				{
						if(RC_Ctl.mouse.press_l==1&&Fricition_Ready)
						{					
								Key_Shoot=1;	
								Shoot_Effect=1;
						}			
				}												
				if(Key_Shoot)
				{
						S_Target[SUM] += Bomb_Angle;
						Start_Cir=1;
						Key_Shoot=0;		
				}	
				if(Start_Cir)
				{
						RM2006_PID_Position_Control();
				}
		}
		else if(Rc_flag)//ң�������� �ٶȻ�
		{
				Start_Cir=0;
				if(RC_Ctl.rc.s1==RC_SW_DOWN&&Fricition_Ready)
				{
						S_Target[SPEED]=3300;
				}
				else
				{
						S_Target[SPEED]=0;
						S_ITerm[SPEED]=0;
						S_PIDTerm=0;
				}
				RM2006_PID_Speed_Control();
		}
}

//�Ӿ�
bool Shoot_Allow,Next_flag=1,Receive_flag,Alone_flag=0,Manual_Effect=1;
int Record_Num;
int32_t Sudoku_Record[2][2];

void Sudoku_Shoot_Control() //�Ź���
{			
		if(RC_Ctl.mouse.press_l==1)//����궨 
		{
				switch(Record_Num)
				{
						case 0:	Sudoku_Record[0][PITCH] = pitch*10;//���Ͻǵĵ�
										Sudoku_Record[0][YAW] = Moto_angle[YAW];
										Pitch_Target[G] = 0;
										Yaw_Target[M] = Y_Mid;										
										Record_Num=1;break;
						case 2:	Sudoku_Record[1][PITCH] = pitch*10;//���½ǵĵ�
										Sudoku_Record[1][YAW] = Moto_angle[YAW];
										Pitch_Target[G] = (Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;
										Yaw_Target[M] = (Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;
										Record_Num=3;break;
				}
		}
		else 
		{
				if(Record_Num==1)
				{
						Record_Num=2;
				}
		}		

		if(Record_Num==3)
		{
				if(Alone_flag&&Next_flag&&System_Mode!=ManualBuff_Mode)//������
				{
						Sudoku_Num=(unsigned char)((rand() % 9 + 1)+48);
						Next_flag=0;
				}
				else if(System_Mode==ManualBuff_Mode)//�ִ�С��
				{	
						if(Manual_Effect)
						{
								if((RC_Ctl.key.v & 0x40) == KEY_PRESSED_OFFSET_Q)
								{
										Sudoku_Num='1';//ע������Ϊunsigned char
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x01) == KEY_PRESSED_OFFSET_W)
								{
										Sudoku_Num='2';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x80) == KEY_PRESSED_OFFSET_E)
								{
										Sudoku_Num='3';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x04) == KEY_PRESSED_OFFSET_A)
								{
										Sudoku_Num='4';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x02) == KEY_PRESSED_OFFSET_S)
								{
										Sudoku_Num='5';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x08) == KEY_PRESSED_OFFSET_D)
								{
										Sudoku_Num='6';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x800) == KEY_PRESSED_OFFSET_Z)
								{
										Sudoku_Num='7';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x1000) == KEY_PRESSED_OFFSET_X)
								{
										Sudoku_Num='8';
										Manual_Effect=0;
								}
								else if((RC_Ctl.key.v & 0x2000) == KEY_PRESSED_OFFSET_C)
								{
										Sudoku_Num='9';
										Manual_Effect=0;
								}	
						}
						else if((RC_Ctl.key.v & 0x40) != KEY_PRESSED_OFFSET_Q&&(RC_Ctl.key.v & 0x01)  != KEY_PRESSED_OFFSET_W&&(RC_Ctl.key.v & 0x80)  != KEY_PRESSED_OFFSET_E&&
										(RC_Ctl.key.v & 0x04) != KEY_PRESSED_OFFSET_A&&(RC_Ctl.key.v & 0x02)  != KEY_PRESSED_OFFSET_S&&(RC_Ctl.key.v & 0x08)  != KEY_PRESSED_OFFSET_D&&
										(RC_Ctl.key.v & 0x800)!= KEY_PRESSED_OFFSET_Z&&(RC_Ctl.key.v & 0x1000)!= KEY_PRESSED_OFFSET_X&&(RC_Ctl.key.v & 0x2000)!= KEY_PRESSED_OFFSET_C&&!Manual_Effect)
						{
								Manual_Effect=1;
						}
				}						
				switch(Sudoku_Num)
				{
//						case '1':Yaw_Target[M] = Sudoku_Record[0][YAW];														Pitch_Target[M] = Sudoku_Record[0][PITCH];	Shoot_Allow=1;break;
//						case '2':Yaw_Target[M] =(Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;Pitch_Target[M] = Sudoku_Record[0][PITCH];	Shoot_Allow=1;break;
//						case '3':Yaw_Target[M] = Sudoku_Record[1][YAW];														Pitch_Target[M] = Sudoku_Record[0][PITCH];	Shoot_Allow=1;break;
//						case '4':Yaw_Target[M] = Sudoku_Record[0][YAW];														Pitch_Target[M] =(Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;	Shoot_Allow=1;break;
//						case '5':Yaw_Target[M] =(Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;Pitch_Target[M] =(Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;	Shoot_Allow=1;break;
//						case '6':Yaw_Target[M] = Sudoku_Record[1][YAW];														Pitch_Target[M] =(Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;	Shoot_Allow=1;break;
//						case '7':Yaw_Target[M] = Sudoku_Record[0][YAW];														Pitch_Target[M] = Sudoku_Record[1][PITCH];	Shoot_Allow=1;break;
//						case '8':Yaw_Target[M] =(Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;Pitch_Target[M] = Sudoku_Record[1][PITCH];	Shoot_Allow=1;break;
//						case '9':Yaw_Target[M] = Sudoku_Record[1][YAW];														Pitch_Target[M] = Sudoku_Record[1][PITCH];	Shoot_Allow=1;break;
						case '1':Yaw_Target[M] = Sudoku_Record[0][YAW];														Pitch_Target[G] = Sudoku_Record[0][PITCH];	Shoot_Allow=1;break;
						case '2':Yaw_Target[M] =(Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;Pitch_Target[G] = Sudoku_Record[0][PITCH];	Shoot_Allow=1;break;
						case '3':Yaw_Target[M] = Sudoku_Record[1][YAW];														Pitch_Target[G] = Sudoku_Record[0][PITCH];	Shoot_Allow=1;break;
						case '4':Yaw_Target[M] = Sudoku_Record[0][YAW];														Pitch_Target[G] =(Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;	Shoot_Allow=1;break;
						case '5':Yaw_Target[M] =(Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;Pitch_Target[G] =(Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;	Shoot_Allow=1;break;
						case '6':Yaw_Target[M] = Sudoku_Record[1][YAW];														Pitch_Target[G] =(Sudoku_Record[0][PITCH] + Sudoku_Record[1][PITCH])/2;	Shoot_Allow=1;break;
						case '7':Yaw_Target[M] = Sudoku_Record[0][YAW];														Pitch_Target[G] = Sudoku_Record[1][PITCH];	Shoot_Allow=1;break;
						case '8':Yaw_Target[M] =(Sudoku_Record[0][YAW] + Sudoku_Record[1][YAW])/2;Pitch_Target[G] = Sudoku_Record[1][PITCH];	Shoot_Allow=1;break;
						case '9':Yaw_Target[M] = Sudoku_Record[1][YAW];														Pitch_Target[G] = Sudoku_Record[1][PITCH];	Shoot_Allow=1;break;
				}
		}
}
int buf_num;
u8 Buf[20];
bool Gimbal_Arrived;
uint32_t Time_record[20];
extern int Sudoku_First;
void Auto_Shoot_Control()      
{	
		Sudoku_Shoot_Control();		
		if(abs(Angle_Error[YAW])<=150&&abs(Angle_Error[PITCH])<=100&&Shoot_Allow)
		{
//				Buf[buf_num] = Sudoku_Num;  //������
//				Time_record[buf_num] = micros();
//				buf_num++;
//				if(buf_num>20)
//						buf_num=0;
				Sudoku_Num=0;			//��0�Ӿ�����
				Shoot_Allow=0;		//�ɹ��������� �������
				Gimbal_Arrived=1;	//��̨����ָ��Ŀ��
		}
		else
				Gimbal_Arrived=0;
		if(Gimbal_Arrived)
		{
				S_Target[SUM] += Bomb_Angle;	
				Gimbal_Arrived=0;
				Start_Cir=1;		
				Next_flag=0;  	  //������������λ
		}		
		if(Start_Cir)
		{
				RM2006_PID_Position_Control();
		}
}

void Weapon_Control()
{
		if(Start_Mode)//����ģʽ ��̨��еģʽ���� ���̾�ֹ
		{
				System_Start();							
		}
		else
		{
				Get_System_Mode();	//��ȡ��ǰϵͳģʽ						
				Friction_Control(); //Ħ����	
				Arsenal_Control();	//����
				Gimbal_Remote(); 		//��̨����ָ��
				if(System_Mode == BigBuff_Mode||System_Mode == SmallBuff_Mode||System_Mode == ManualBuff_Mode)
				{
						Auto_Shoot_Control();//�Զ����
				}
				else
				{						
						Shoot_Control();//�ֶ����	
						Laser_On;				//���⼤��
						Sudoku_First=0;
						Record_Num=0;
						Sudoku_Num=0;
						Sudoku_Record[0][PITCH]=0;
						Sudoku_Record[1][PITCH]=0;
						Sudoku_Record[0][YAW]=0;
						Sudoku_Record[1][YAW]=0;
				}			
		}			
		Gimbal_Control(); //��̨����
}

//ʧ�ر���
//int SystemMonitor=Error_Mode;
int SystemMonitor=Normal_Mode;

int Out_Control_Protection()
{
		System_Num++;
		if(System_Num<=0)
		{
				System_Num=1000;
		}
		if((System_Num>=200)||(Protect_flag))
		{
				return Error_Mode;
		}
		else 
		{
				return Normal_Mode;
		}
}

void Stop(void)
{
//		Start_Mode=1;				  //����ģʽ
	
		Laser_Off;
		Arsenal_Open_flag=0;	//����
		Servo_PWM(Close_Angle);	
	
		Friction_ON=0;				//Ħ����
		Friction_Switch=0;
		Fricition_Ready=0;
		Friction_Number=0;
		Friction_PWM(0,0);
											
		Key_Shoot=0;		//�͵����
		Shoot_Allow=0;
		S_Target[SPEED]=0;
		S_Target[POSITION] = RM06_Angle.Total;
		S_PIDTerm=0;
			
		if(!Start_Mode)
		{
				Yaw_Target[G] = yaw*10;
				Yaw_Target[M] = Y_Mid;
		}
		Gimbal_PIDTerm[PITCH]=0;	//��̨		
		Gimbal_PIDTerm[YAW]=0;
	
		Rocking_flag=0;
		Rocking_Num=0;
		Chassis_PIDTerm[0]=0;	//����
		Chassis_PIDTerm[1]=0;
		Chassis_PIDTerm[2]=0;
		Chassis_PIDTerm[3]=0;
}
