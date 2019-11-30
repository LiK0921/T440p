#include "can1.h"

int16_t Motor_chassis[4][2],Moto_angle[2],Moto_realCurrent[2];	//机械角度	0~8191(0x1FFF)
int16_t Chassis_Err[4];
Measure_t Measure;
Judge_t Judge;

int Chassis_Monitor[4];
bool Motor_Connect[4];
extern float Motor_Speed[4];

void CAN1_Init()
{
		GPIO_InitTypeDef gpio;
	  NVIC_InitTypeDef nvic;
		CAN_InitTypeDef can;
		CAN_FilterInitTypeDef filter;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
		
		gpio.GPIO_Mode=GPIO_Mode_AF;
	  gpio.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_11;
		GPIO_Init(GPIOA,&gpio);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
		nvic.NVIC_IRQChannel=CAN1_TX_IRQn;
	  nvic.NVIC_IRQChannelPreemptionPriority=1;
	  nvic.NVIC_IRQChannelSubPriority=1;
		nvic.NVIC_IRQChannelCmd=ENABLE;
	  NVIC_Init(&nvic);
	
	  nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
		can.CAN_TTCM = DISABLE;		//非时间触发通信模式
    can.CAN_ABOM = DISABLE;		//软件自动离线管理
    can.CAN_AWUM = DISABLE;		//睡眠模式通过软件唤醒(清楚CAN->MCR的SLEEP位)
    can.CAN_NART = DISABLE;		//禁止报文自动传送 若七个电机接一个CAN 会影响发送 此时可改为ENABLE
    can.CAN_RFLM = DISABLE;		//报文不锁定，新的覆盖旧的
    can.CAN_TXFP = ENABLE;		//优先级由报文标识符决定
		can.CAN_BS1=CAN_BS1_9tq;
		can.CAN_BS2=CAN_BS2_4tq;
		can.CAN_Mode=CAN_Mode_Normal;
		can.CAN_Prescaler=3;
		can.CAN_SJW=CAN_SJW_1tq;
		CAN_Init(CAN1,&can);
		
		filter.CAN_FilterNumber=0;  							 			//过滤器0
		filter.CAN_FilterMode=CAN_FilterMode_IdMask;   	//屏蔽模式
		filter.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位宽
		filter.CAN_FilterFIFOAssignment=0;              //过滤器0关联到FIFO0
		filter.CAN_FilterActivation=ENABLE;   				  //激活过滤器
		filter.CAN_FilterIdHigh=0x0000;                 //32位ID
		filter.CAN_FilterIdLow=0x0000;
		filter.CAN_FilterMaskIdHigh=0x0000;             //32位Mask
		filter.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInit(&filter);
		
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);    ////FIFO0消息挂号中断允许
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}



void CAN1_Send_Chassis_Motor(void)		//底盘电机
{		
		CanTxMsg TxMessage1;
	
		Chassis_Monitor[0]++;Chassis_Monitor[1]++;
		Chassis_Monitor[2]++;Chassis_Monitor[3]++;
		if(Chassis_Monitor[0]>=50||Chassis_Monitor[1]>=50)//底盘电机通讯检测 若一电机故障 则一同注释对应的电机输出 保持麦轮对称性
		{
				Chassis_Monitor[0]=1000;Chassis_Monitor[1]=1000;
				Chassis_PIDTerm[0]=0;Chassis_PIDTerm[1]=0;
		}
		if(Chassis_Monitor[2]>=50||Chassis_Monitor[3]>=50)
		{
				Chassis_Monitor[2]=1000;Chassis_Monitor[3]=1000;
				Chassis_PIDTerm[2]=0;Chassis_PIDTerm[3]=0;
		}
		
		TxMessage1.StdId = 0x200;					 //使用的扩展ID
		TxMessage1.IDE = CAN_ID_STD;				 //标准模式
		TxMessage1.RTR = CAN_RTR_DATA;			 //数据帧RTR=0，远程帧RTR=1
		TxMessage1.DLC = 8;							 	 //数据长度为8字节

		TxMessage1.Data[0] = (unsigned char)((int16_t)Chassis_PIDTerm[0]>>8);
		TxMessage1.Data[1] = (unsigned char)((int16_t)Chassis_PIDTerm[0]);
		TxMessage1.Data[2] = (unsigned char)((int16_t)Chassis_PIDTerm[1]>>8);
		TxMessage1.Data[3] = (unsigned char)((int16_t)Chassis_PIDTerm[1]);
		TxMessage1.Data[4] = (unsigned char)((int16_t)Chassis_PIDTerm[2]>>8);
		TxMessage1.Data[5] = (unsigned char)((int16_t)Chassis_PIDTerm[2]);
		TxMessage1.Data[6] = (unsigned char)((int16_t)Chassis_PIDTerm[3]>>8);
		TxMessage1.Data[7] = (unsigned char)((int16_t)Chassis_PIDTerm[3]);

		CAN_Transmit(CAN1, &TxMessage1);
}

void CAN1_Send_Gimbal_Motor(void)				//云台
{   
		CanTxMsg TxMessage;
		TxMessage.StdId = 0x1FF;					 //使用的扩展ID
		TxMessage.IDE = CAN_ID_STD;				 //标准模式
		TxMessage.RTR = CAN_RTR_DATA;			 //数据帧RTR=0，远程帧RTR=1
		TxMessage.DLC = 8;							 	 //数据长度为8字节
	 
	  TxMessage.Data[0]=(unsigned char)((int16_t)Gimbal_PIDTerm[YAW]>>8);    //Yaw
	  TxMessage.Data[1]=(unsigned char)((int16_t)Gimbal_PIDTerm[YAW]);
		TxMessage.Data[2]=(unsigned char)((int16_t)Gimbal_PIDTerm[PITCH]>>8);  //Pitch                     
		TxMessage.Data[3]=(unsigned char)((int16_t)Gimbal_PIDTerm[PITCH]);
		TxMessage.Data[4]=0;//(unsigned char)((int16_t)S_PIDTerm>>8);      //RM2006
		TxMessage.Data[5]=0;//(unsigned char)((int16_t)S_PIDTerm);
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;	
  
		CAN_Transmit(CAN1,&TxMessage);			
}

void CAN1_RX0_IRQHandler()
{
		CanRxMsg RxMessage;
		if(CAN_GetITStatus!=RESET)
		{
			CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
			CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		}
		if(RxMessage.StdId == 0x199)		
		{    
			Measure.Power = (int16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);    
			Judge.Joule   = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);
			Judge.outV    = (int16_t)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);
			Judge.outA    = (int16_t)(RxMessage.Data[6]<<8 | RxMessage.Data[7]);		
			Judge.Power   = ((float)Judge.outV*(float)Judge.outA/10000);
			if(Measure.Power!=0)
			{
					Measure.Update_flag=1;
					Measure.Num=0;
			}
		}
		if(RxMessage.StdId == 0x201)//左前		
		{   
				Motor_chassis[0][0] = (int16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);		//机械角度	0~8191(0x1FFF)
				Motor_chassis[0][1] = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);		//实际电流转矩测量值	-13000 ~ 13000
				Chassis_Monitor[0]=0;
		}
		if(RxMessage.StdId == 0x202)//右前		
		{      
				Motor_chassis[1][0] = (int16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);		//机械角度	0~8191(0x1FFF)
				Motor_chassis[1][1] = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);		//实际电流转矩测量值	-13000 ~ 13000			
				Chassis_Monitor[1]=0;
		}
		if(RxMessage.StdId == 0x203)//左后
		{     
				Motor_chassis[2][0] = (int16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);		//机械角度	0~8191(0x1FFF)
				Motor_chassis[2][1] = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);		//实际电流转矩测量值	-13000 ~ 13000
				Chassis_Monitor[2]=0;
		}
		if(RxMessage.StdId == 0x204)//右后
		{   
				Motor_chassis[3][0] = (int16_t)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);		//机械角度	0~8191(0x1FFF)
				Motor_chassis[3][1] = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);		//实际电流转矩测量值	-13000 ~ 13000			
				Chassis_Monitor[3]=0;
		}	
		if(RxMessage.StdId == 0x205)			//YAW
		{
				Moto_angle[YAW]=(int16_t)(RxMessage.Data[0]<<8|RxMessage.Data[1]);					  //机械角度	0~8191(0x1FFF)
				Moto_realCurrent[YAW] = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);		//实际电流转矩测量值	-13000 ~ 13000
		}	
		if(RxMessage.StdId == 0x206)			//Pitch
		{
				Moto_angle[PITCH]=(int16_t)(RxMessage.Data[0]<<8|RxMessage.Data[1]);					  //机械角度	0~8191(0x1FFF)
				Moto_realCurrent[PITCH] = (int16_t)(RxMessage.Data[2]<<8 | RxMessage.Data[3]);		//实际电流转矩测量值	-13000 ~ 13000
		}
}

void CAN1_TX_IRQHandler()
{
	if(CAN_GetITStatus!=RESET)
		{
			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		}
}


