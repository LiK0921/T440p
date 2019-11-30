#include "uart5.h"
 
void uart5_Init(void)   
{
  USART_InitTypeDef uart5;
	GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); 
	
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD,&gpio);

	uart5.USART_BaudRate = 115200;   
	uart5.USART_WordLength = USART_WordLength_8b;
	uart5.USART_StopBits = USART_StopBits_1;
	uart5.USART_Parity = USART_Parity_No;
	uart5.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  uart5.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART5,&uart5);
	
	USART_Cmd(UART5,ENABLE); 
	USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);  
	
	nvic.NVIC_IRQChannel = UART5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 3;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic); 
}

void UART5_IRQHandler(void)
{
		if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
		{
				USART_ReceiveData(UART5);
		}	
		rxdata =USART_ReceiveData(UART5);
		Sudoku_Data_Receive();
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
}

int Sudoku_First=0;
u8 rxdata,rx_buf[2];
extern bool Receive_flag;
extern int Receive_Num;
unsigned char Sudoku_Num;

//大符
void Sudoku_Data_Receive() //注意陈妍的妙算串口驱动是用C/C++写的 每次发9位 除了特定的字符外别的都是乱码 而张跃的是用Python写的 没有固定位数 因此没有乱码问题
{
		if(System_Mode != ManualBuff_Mode)
		{
				rx_buf[Sudoku_First]=rxdata;
				if(rx_buf[0]=='A')
				{
						if(rx_buf[1]>=0x31 && rx_buf[1]<=0x39)
						{
								Sudoku_Num = rx_buf[1];
								Receive_flag=1;
								Receive_Num=0;
						}
				}
				Sudoku_First++;
				if(Sudoku_First>=2)
				{
						rx_buf[0]=0;
						rx_buf[1]=0;
						Sudoku_First=0;
				}
		}
}

void UART5_Init(u32 bound)
{
  // GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE); 
	
	// 串口5对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
	
	// UART5端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

  // UART5初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART5, &USART_InitStructure); //初始化串口5
	
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启相关中断
  USART_Cmd(UART5, ENABLE);  //使能串口1
	USART_ClearFlag(UART5, USART_FLAG_TC);
	
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

void UART5_sendChar(u8 c)
{
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET); 
	USART_SendData(UART5,c);   
} 

/***********************************************************************/
/*                        Functions		                                 */
/***********************************************************************/
void UART5_sendDataToPrint(int16_t dax, int16_t day, int16_t daz, int16_t dgx, int16_t dgy, int16_t dgz, int16_t dmx, int16_t dmy, int16_t dmz)
{
	uint8_t i;
  uint16_t temp=0;
  uint8_t data_to_send[24];
     
  data_to_send[0] = 0xAA;
  data_to_send[1] = 0xAA;
  data_to_send[2] = 0x02;
  data_to_send[3] = 18;
	
	data_to_send[4] = (dax & 0xFF00) >> 8;
	data_to_send[5] = dax & 0xFF;
	data_to_send[6] = (day & 0xFF00) >> 8;
	data_to_send[7] = day & 0xFF;
	data_to_send[8] = (daz & 0xFF00) >> 8;
	data_to_send[9] = daz & 0xFF;
	data_to_send[10] = (dgx & 0xFF00) >> 8;
	data_to_send[11] = dgx & 0xFF;
	data_to_send[12] = (dgy & 0xFF00) >> 8;
	data_to_send[13] = dgy & 0xFF;
	data_to_send[14] = (dgz & 0xFF00) >> 8;
	data_to_send[15] = dgz & 0xFF;
	data_to_send[16] = (dmx & 0xFF00) >> 8;
	data_to_send[17] = dmx & 0xFF;
	data_to_send[18] = (dmy & 0xFF00) >> 8;
	data_to_send[19] = dmy & 0xFF;
	data_to_send[20] = (dmz & 0xFF00) >> 8;
	data_to_send[21] = dmz & 0xFF;

	for(i = 0; i < 22; i++)
	{
		temp += data_to_send[i];
  }
	data_to_send[22] = temp & 0xFF;
	
	for(i = 0; i < 23; i++)
	{
		UART5_sendChar(data_to_send[i]);
	}
}

