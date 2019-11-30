#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "init.h"
#include "schedule.h"

//ALIENTEK 探索者STM32F407开发板 实验0
//STM32F4工程模板-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK


u8 Init_Finish;
int main(void)
{
	Init_Finish = All_Init();
  while(1)
	{
		 Duty_Loop();
	}
}




