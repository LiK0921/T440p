#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "init.h"
#include "schedule.h"

//ALIENTEK ̽����STM32F407������ ʵ��0
//STM32F4����ģ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK


u8 Init_Finish;
int main(void)
{
	Init_Finish = All_Init();
  while(1)
	{
		 Duty_Loop();
	}
}




