/*
编写者：Kevin
修改者1：吴华勃
修改者2：郑小威
编译环境：MDK-Lite  Version: 1.0
初版时间: 2016-1-31
第一次修改时间：2017-2-10
第二次修改时间：2017-4-15
功能：
用STM32F4串口4读取JY61的数据，波特率115200。
接线
STM32Core             JY61
VCC        ----       VCC
GND        ----       GND
RX3        ----       TX
TX3        ----       RX
------------------------------------
 */

#include "jy61.h"

struct STime stcTime;
struct SAcc	stcAcc;  
struct SGyro stcGyro;
struct SAngle stcAngle;
struct SMag stcMag;
struct SDStatus stcDStatus;
struct SPress stcPress;
struct SLonLat stcLonLat;
struct SGPSV stcGPSV;
unsigned char sensor_data;
//float accx,accy,accz;
//float gyrox,gyroy,gyroz;
//float roll ,pitch ,yaw;
float Chassis_pitch;
int trans_success_flag;
//static unsigned char TxBuffer[256];
//static unsigned char TxCounter=0;
//static unsigned char count=0; 

void put_sensor_data(void)
{        
//	printf("Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
//	printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);	
//	printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//	printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
}


//void USART4_Init(void)
//{
//    USART_InitTypeDef uart4;
//		GPIO_InitTypeDef  gpio;
//    NVIC_InitTypeDef  nvic;
//	
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
//		
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); 
//		
//		gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//		gpio.GPIO_Mode = GPIO_Mode_AF;
//    gpio.GPIO_OType = GPIO_OType_PP;
//    gpio.GPIO_Speed = GPIO_Speed_100MHz;
//		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;	//GPIO_PuPd_NOPULL
//		GPIO_Init(GPIOA,&gpio);

//		uart4.USART_BaudRate = 115200;
//		uart4.USART_WordLength = USART_WordLength_8b;
//		uart4.USART_StopBits = USART_StopBits_1;
//		uart4.USART_Parity = USART_Parity_No;
//		uart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
//    uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//		USART_Init(UART4,&uart4);

//    USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
//		USART_Cmd(UART4,ENABLE);
//    
//		nvic.NVIC_IRQChannel = UART4_IRQn;
//		nvic.NVIC_IRQChannelPreemptionPriority = 1;
//		nvic.NVIC_IRQChannelSubPriority = 8;
//		nvic.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&nvic);
//}

void sensor_Data_handle(unsigned char sensor_data)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
    ucRxBuffer[ucRxCnt++] = sensor_data;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
	    return;
	 }
	 if (ucRxCnt<11) 
     {
	    trans_success_flag = 0;
		return;
	}//数据不满11个，则返回
	 else
	    {
		    switch(ucRxBuffer[1])
	    	{
		    	case 0x50:	memcpy(&stcTime ,&ucRxBuffer[2] ,8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
		    	case 0x51:	memcpy(&stcAcc ,&ucRxBuffer[2] ,8);break;
			    case 0x52:	memcpy(&stcGyro ,&ucRxBuffer[2] ,8);break;
			    case 0x53:	memcpy(&stcAngle ,&ucRxBuffer[2] ,8);break;
			    case 0x54:	memcpy(&stcMag ,&ucRxBuffer[2] ,8);break;
		    	case 0x55:	memcpy(&stcDStatus ,&ucRxBuffer[2] ,8);break;
		    	case 0x56:	memcpy(&stcPress ,&ucRxBuffer[2] ,8);break;
		    	case 0x57:	memcpy(&stcLonLat ,&ucRxBuffer[2] ,8);break;
		    	case 0x58:	memcpy(&stcGPSV ,&ucRxBuffer[2] ,8);break;
				default:break;
		    }
			ucRxCnt=0;
//			accx = ((float)stcAcc.a[0])/32768*16;
//			accy = ((float)stcAcc.a[1])/32768*16;
//			accz = ((float)stcAcc.a[2])/32768*16;		
//			gyrox = ((float)stcGyro.w[0])/32768*2000;
//			gyroy = ((float)stcGyro.w[1])/32768*2000;
//			gyroz = ((float)stcGyro.w[2])/32768*2000;
			Chassis_pitch = ((float)stcAngle.Angle[0])/32768*180;
//			roll = ((float)stcAngle.Angle[1])/32768*180;
//			yaw = ((float)stcAngle.Angle[2])/32768*180;
            trans_success_flag = 1;
	    }
}


