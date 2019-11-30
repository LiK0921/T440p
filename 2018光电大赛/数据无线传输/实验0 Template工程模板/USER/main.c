#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "init.h"
#include "key.h"
#include "led.h"
#include "usart.h"

//ALIENTEK 探索者STM32F407开发板 实验0
//STM32F4工程模板-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK


u8 Init_Finish;
int main(void)
{
	u8 key;           //保存键值	
	Init_Finish = All_Init();
  while(1)
	{
		 key=Key_Scan(0);		//得到键值
				if(key)
			{						   
				switch(key)
				{				 
					case WKUP_PRES:	//控制蜂鸣器
						Usart_to_test();
						break;
					case KEY0_PRES:	//控制LED0翻转
						LED0=!LED0;
						break;
					case KEY1_PRES:	//控制LED1翻转	 
						LED1=!LED1;
						break;
					case KEY2_PRES:	//同时控制LED0,LED1翻转 
						LED0=!LED0;
						LED1=!LED1;
						break;
				}
			}else delay_ms(10); 
	}
}




