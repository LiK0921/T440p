#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "init.h"
#include "key.h"
#include "led.h"
#include "usart.h"

//ALIENTEK ̽����STM32F407������ ʵ��0
//STM32F4����ģ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK


u8 Init_Finish;
int main(void)
{
	u8 key;           //�����ֵ	
	Init_Finish = All_Init();
  while(1)
	{
		 key=Key_Scan(0);		//�õ���ֵ
				if(key)
			{						   
				switch(key)
				{				 
					case WKUP_PRES:	//���Ʒ�����
						Usart_to_test();
						break;
					case KEY0_PRES:	//����LED0��ת
						LED0=!LED0;
						break;
					case KEY1_PRES:	//����LED1��ת	 
						LED1=!LED1;
						break;
					case KEY2_PRES:	//ͬʱ����LED0,LED1��ת 
						LED0=!LED0;
						LED1=!LED1;
						break;
				}
			}else delay_ms(10); 
	}
}




