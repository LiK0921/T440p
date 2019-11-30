#include "AnoParameter.h"
#include "drv_w25qxx.h"
#include "parameter.h"
#include "include.h"
#include "data_transfer.h"

_parameter_state_st para_sta;

static void Ano_Parame_Write(void)
{
	All_PID_Init();
  pid_setup.groups.first_init = SOFT_VER;
	
  //先擦除后写入
	Flash_SectorErase ( 0x000000, 1 );							//擦除第一扇区
	Flash_SectorsWrite ( 0x000000, &pid_setup.raw_data[0], 1 );	//将参数写入第一扇区
}

void Ano_Parame_Read(void)
{
	Flash_SectorsRead ( 0x000000, &pid_setup.raw_data[0], 1 );		//读取第一扇区内的参数
	
	if(pid_setup.groups.first_init != SOFT_VER)	//内容没有被初始化，则进行参数初始化工作
	{		
		Para_ResetToFactorySetup();
		Ano_Parame_Write();
	}	
}

u8 write_flag;

void Ano_Parame_Write_task(u16 dT_ms)
{
	//因为写入flash耗时较长，我们飞控做了一个特殊逻辑，在解锁后，是不进行参数写入的，此时会置一个需要写入标志位，等飞机降落锁定后，再写入参数，提升飞行安全性
	//为了避免连续更新两个参数，造成flash写入两次，我们飞控加入一个延时逻辑，参数改变后一秒，才进行写入操作，可以一次写入多项参数，降低flash擦写次数
		if(para_sta.save_trig == 1) 	//如果触发存储标记1
		{		
			para_sta.time_delay = 0;  	//计时复位
			para_sta.save_trig = 2;   	//触发存储标记2
		}
		
		if(para_sta.save_trig == 2) 	//如果触发存储标记2
		{
			if(para_sta.time_delay<1500) //计时小于3000ms
			{
				para_sta.time_delay += dT_ms; //计时
			}
			else
			{
				para_sta.save_trig = 0;  //存储标记复位
				Ano_Parame_Write();      //执行存储
				write_flag = 1;
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}
}

/*PID参数初始化*/
void All_PID_Init(void)
{
	PID_Para_Init();
}
