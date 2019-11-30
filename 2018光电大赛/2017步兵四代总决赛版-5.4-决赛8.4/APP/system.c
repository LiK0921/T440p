#include "system.h"

static volatile uint32_t usTicks = 0;
extern volatile uint32_t sysTickUptime;
uint32_t currentTime = 0;

short gyrox,gyroy,gyroz;	//陀螺仪原始数据
float pitch,roll,yaw;			//欧拉角
float pitch1,roll1,yaw1;

//系统启动模式
extern bool Start_Mode;
extern void System_Start(void);

//获取运动模式
extern void Get_System_Mode(void);

//云台
extern void Gimbal_Remote(void);//云台操作指令
extern void Gimbal_Control(void);

//底盘
extern int Rocking_Num;
extern bool Roam_Shoot,Climb_flag;
extern float Current_P,Motor_Speed[4],Power_Num;
extern void Chassis_Control(void);	

//弹仓
extern bool Arsenal_Open_flag,Infrared_Interval,Infrared_Add,Infrared_Effect;
extern int Close_Num;

//摩擦轮
extern bool Friction_ON,Fricition_Ready;
extern int Friction_Number;

//武器
extern void Weapon_Control(void);
extern bool Next_flag,Shoot_Effect,Receive_flag;

//裁判系统
extern Measure_t Measure;
extern Judge_t   Judge;

//失控保护
extern int SystemMonitor;
extern int Out_Control_Protection(void);
extern void Stop(void);

PID_t PID;

float constrain(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int constrain_int(int amt,int low,int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

//计数器初始化
static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000; 
}

//以微秒为单位返回系统时间
uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do {
			ms = sysTickUptime;
			cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//微秒级延时
void delay_us(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}

//毫秒级延时
void delay_ms(uint32_t ms)
{
	while (ms--)
			delay_us(1000);
}

//以毫秒为单位返回系统时间
uint32_t millis(void)
{
	return sysTickUptime;
}

void Parameter_Init(void)
{  
		//Pitch 
		//陀螺仪模式参数
		PID.Gimbal.P[PITCH][G]=3.2;		
		PID.Gimbal.I[PITCH][G]=10;			
	  PID.Gimbal.Level[PITCH][G]=1;	
	
		//机械模式参数
		PID.Gimbal.P[PITCH][M]=1.6;
		PID.Gimbal.I[PITCH][M]=5;	
	  PID.Gimbal.Level[PITCH][M]=1;
		
		//自动模式参数
		PID.Gimbal.P[PITCH][A]=4;
		PID.Gimbal.I[PITCH][A]=15;
		PID.Gimbal.Level[PITCH][A]=1;
		
		//单级
		PID.Gimbal.P[PITCH][S]=15;
		PID.Gimbal.I[PITCH][S]=1; 
		
		//Yaw
		//陀螺仪模式参数
		PID.Gimbal.P[YAW][G]=3.8;   
		PID.Gimbal.I[YAW][G]=6;		
	  PID.Gimbal.Level[YAW][G]=15;
	
		//机械模式参数
		PID.Gimbal.P[YAW][M]=3.2;
		PID.Gimbal.I[YAW][M]=5;	
	  PID.Gimbal.Level[YAW][M]=12;
		
		//自动模式参数
		PID.Gimbal.P[YAW][A]=4;
		PID.Gimbal.I[YAW][A]=5;
		PID.Gimbal.Level[YAW][A]=15;
		
		//单级
		PID.Gimbal.P[YAW][S]=15;
		PID.Gimbal.I[YAW][S]=1;
		
		PID.Gimbal.IMax=1900;	 

		//底盘
		PID.Speed.P=2;       
		PID.Speed.I=20; 
		PID.Speed.IMax=3000; 
		
		//RM2006	
		PID.RM2006.P[POSITION]=0.2;
		PID.RM2006.I[POSITION]=1;
		PID.RM2006.D[POSITION]=0.02;
		PID.RM2006.IMax[POSITION]=2000;
		
		PID.RM2006.P[SPEED]=7;
		PID.RM2006.I[SPEED]=2;
		PID.RM2006.IMax[SPEED]=1000;
		
		PID.Looptime=0.002;
}

int pass_num;
bool pass_flag=1;
void System_Init(void)
{		
		static uint32_t loopTime_mpu6050 = 0;	
	  cycleCounterInit();
	  SysTick_Config(SystemCoreClock / 1000);
	  Parameter_Init();
		Laser_Init();			//红外激光
		Led_Init();				//Led指示灯
		CAN1_Init();			//云台和底盘通讯
		CAN2_Init();    	//拨弹电机通讯
		TIM1_Init();			//舵机		
//		TIM3_Init();		 	//拨弹电机和弹仓舵机
		TIM4_Init();		 	//摩擦轮
		usart2_Init();   	//遥控
//		usart3_Init(); 	 	//外部陀螺仪
//		uart4_Init();	   	//裁判系统	
		uart5_Init();		 	//视觉
//		UART5_Init(38400);
		Infrared_Init(); 	//补弹 红外发射
		delay_ms(200);
		MPU_Init();
		while(mpu_dmp_init())//注意自检函数
		{
				currentTime = micros();//获取当前系统时间	
				if((int32_t)(currentTime - loopTime_mpu6050) >= 100000)  
				{	
						loopTime_mpu6050 = currentTime + 100000;			//100ms
						pass_num++;
						if(pass_num>=3)//若超时 则屏蔽自检函数
						{
								pass_flag=0;
								pass_num=10;
						}
				}
		}	
}

int Protect_Num,Infrared_Num,BombInit,Receive_Num,BuffChoice,Climb_Num,Static_Num;
bool Protect_flag=0,Game_flag=0,CAN_Open[3]={1,1,1},Target_Static;
//主循环
void Loop(void)
{	
		static uint32_t currentTime  				 = 0;
		static uint32_t loopTime_chassis 		 = 0;
		static uint32_t loopTime_weapon 		 = 0;
		static uint32_t loopTime_20ms 			 = 0;
		static uint32_t loopTime_Shoot 			 = 0;
		static uint32_t loopTime_1s 				 = 0;
		static uint32_t loopTime_Infrared		 = 0;
		static uint32_t loopTime_AloneBuff   = 0;
		static uint32_t loopTime_BombInit 	 = 0;
		static uint32_t loopTime_Buff_Printf = 0;
		static uint32_t loopTime_Buff_Led 	 = 0;
		static uint32_t loopTime_Climb 			 = 0;
		static uint32_t loopTime_Static 		 = 0;
		currentTime = micros();	//获取当前系统时间
		
		if((int32_t)(currentTime - loopTime_chassis) >= 0)  
		{			
				loopTime_chassis = currentTime + 1000;		//1ms			
				SystemMonitor=Out_Control_Protection();		//失控保护检查		
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//读取加速度
				mpu_dmp_get_data(&roll,&pitch,&yaw);			//读取欧拉角
				if(SystemMonitor==Error_Mode&&!Game_flag)	//失控保护
				{
						Stop();
						Protect_Num++;
						if(Protect_Num>200)
						{
								Protect_Num=0;
								Protect_flag=0;	
						}						
				}
				if(SystemMonitor==Normal_Mode||Game_flag)					 
				{		
						if(!Start_Mode)
						{	
								Chassis_Control();//底盘驱动
						}
				}		
				if(!CAN_Open[0])
				{
						Chassis_PIDTerm[0]=0;
						Chassis_PIDTerm[1]=0;
						Chassis_PIDTerm[2]=0;
						Chassis_PIDTerm[3]=0;
				}
				CAN1_Send_Chassis_Motor();
		}
		if((int32_t)(currentTime - loopTime_weapon) >= 0)  
		{			
				loopTime_weapon = currentTime + 2000;	//2ms	
				if(SystemMonitor==Normal_Mode||Game_flag)					 
				{	
						Weapon_Control();//武器系统	
				}
				if(!CAN_Open[1])
				{
						Gimbal_PIDTerm[PITCH]=0;Gimbal_PIDTerm[YAW]=0;
				}
				if(!CAN_Open[2])
						S_PIDTerm=0;
				CAN1_Send_Gimbal_Motor();	
				CAN2_Send_RM2006_Motor();
		}
		if((int32_t)(currentTime - loopTime_20ms) >= 0)  
		{	
				loopTime_20ms = currentTime + 20000;//20ms
//				UART5_sendDataToPrint(0,(int16_t)(Judge.Joule*100),(int16_t)(Judge.Power*100),0,0
//														 ,(int16_t)(realPower*100),0,0,0);
				if(Infrared_Effect)//红外发射端60ms发射一次
				{
						Infrared_Num++;
						if(Infrared_Num>2)
						{
								Infrared_Add=1;
								Infrared_Num=10;
						}
				}
				else
						Infrared_Num=0;
				
				if(!Measure.Update_flag)//功率板掉线保护
				{
						Measure.Num++;												
						if(Measure.Num>50)
						{
							Measure.Num=100;
							Blue_Off;
						}
				}
				else
						Blue_On;//功率板通讯正常 数据正常 指示灯常亮
		}
		if(Receive_flag)//视觉识别到 Led亮 持续2s
		{
				if((int32_t)(currentTime - loopTime_Buff_Led) >= 1000000)  
				{
						loopTime_Buff_Led = currentTime + 1000000;        //1s
						Receive_Num++;
						if(Receive_Num>=2)
						{
								Receive_flag=0;
						}
				}					
		}
		else
				loopTime_Buff_Led = currentTime;
		if(Receive_flag)
				Visual_On;
		else
				Visual_Off;
					
		if((int32_t)(currentTime - loopTime_BombInit) >= 0)
		{
				loopTime_BombInit = currentTime +500000;//500ms		
				if(BombInit>=0)
				{
						BombInit++;	
				}
				if(BombInit>6)//拨弹电机位置环初始化  
				{
						BombInit=-1; 
						RM06_Angle.Begin = RM06_Motor[0]; 
				}
	//			if(BombInit == -1)//调试专用			
	//			{				
	//				S_Target[SUM] += Bomb_Angle;			
	//			}
		}
		
		if(Shoot_Effect)//限制射频
		{
				if((int32_t)(currentTime - loopTime_Shoot) >= 220000)  
				{	
						loopTime_Shoot = currentTime + 220000;        //220ms
						Shoot_Effect=0;
				}
		}
		else
				loopTime_Shoot = currentTime;
		
		if((int32_t)(currentTime - loopTime_1s) >= 0)  
		{
				loopTime_1s = currentTime + 1000000;        //1s
				if(Friction_ON)//摩擦轮开启2s后才可射击
				{
						Friction_Number++;
						if(Friction_Number>1)
						{
								Fricition_Ready=1;   
								Friction_Number=4;
						}			
				}						
				if(Arsenal_Open_flag)//弹仓关闭倒计时
				{	
						Close_Num++;					
				}
		}
		
		if(Infrared_Effect)//按键R有效性间隔800ms 红外发射持续800ms
		{
				if((int32_t)(currentTime - loopTime_Infrared) >= 800000)  
				{
						loopTime_Infrared = currentTime + 800000;        //800ms
						Infrared_Effect=0;
				}
		}
		else
				loopTime_Infrared = currentTime;
		
		if((int32_t)(currentTime - loopTime_AloneBuff) >= 0)  
		{			
				loopTime_AloneBuff = currentTime + 500000;			//500ms			
				Next_flag=1;//单机版大符打击间隔
		}
		
		if((int32_t)(currentTime - loopTime_Buff_Printf) >= 0) //大小符选择交互 
		{			
				loopTime_Buff_Printf = currentTime + 10000;			//10ms	
				if(System_Mode == SmallBuff_Mode)
				{
						BuffChoice=1;					
				}
				else if(System_Mode==BigBuff_Mode)
				{
						BuffChoice=2;						
				}
				if(BuffChoice==1)
				{
						printf("S");
				}
				else if(BuffChoice==2)
				{
						printf("B");
				}
		}			
		
		if(Climb_flag)//手动爬坡模式 持续5s
		{
				if((int32_t)(currentTime - loopTime_Climb) >= 5000000)  
				{			
						loopTime_Climb = currentTime + 5000000;		//5s	
						Climb_Num++;
						if(Climb_Num>=1)
								Climb_flag=0;
				}		
		}		
		else
				loopTime_Climb = currentTime;
		
		if(RC_Ctl.mouse.x==0&&RC_Ctl.mouse.y==0&&RC_Ctl.rc.ch0==RC_CH_VALUE_OFFSET&&RC_Ctl.rc.ch1==RC_CH_VALUE_OFFSET)//云台静止2s后进入yaw轴防漂状态
		{
				if((int32_t)(currentTime - loopTime_Static >= 2000000))  
				{
						loopTime_Static = currentTime + 2000000;		//2s	
						Static_Num++;
						if(Static_Num>=1)
								Target_Static=1;
				}
		}
		else
		{
				loopTime_Static = currentTime;
				Target_Static=0;
		}
}
