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
	
  //�Ȳ�����д��
	Flash_SectorErase ( 0x000000, 1 );							//������һ����
	Flash_SectorsWrite ( 0x000000, &pid_setup.raw_data[0], 1 );	//������д���һ����
}

void Ano_Parame_Read(void)
{
	Flash_SectorsRead ( 0x000000, &pid_setup.raw_data[0], 1 );		//��ȡ��һ�����ڵĲ���
	
	if(pid_setup.groups.first_init != SOFT_VER)	//����û�б���ʼ��������в�����ʼ������
	{		
		Para_ResetToFactorySetup();
		Ano_Parame_Write();
	}	
}

u8 write_flag;

void Ano_Parame_Write_task(u16 dT_ms)
{
	//��Ϊд��flash��ʱ�ϳ������Ƿɿ�����һ�������߼����ڽ������ǲ����в���д��ģ���ʱ����һ����Ҫд���־λ���ȷɻ�������������д��������������а�ȫ��
	//Ϊ�˱������������������������flashд�����Σ����Ƿɿؼ���һ����ʱ�߼��������ı��һ�룬�Ž���д�����������һ��д��������������flash��д����
		if(para_sta.save_trig == 1) 	//��������洢���1
		{		
			para_sta.time_delay = 0;  	//��ʱ��λ
			para_sta.save_trig = 2;   	//�����洢���2
		}
		
		if(para_sta.save_trig == 2) 	//��������洢���2
		{
			if(para_sta.time_delay<1500) //��ʱС��3000ms
			{
				para_sta.time_delay += dT_ms; //��ʱ
			}
			else
			{
				para_sta.save_trig = 0;  //�洢��Ǹ�λ
				Ano_Parame_Write();      //ִ�д洢
				write_flag = 1;
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}
}

/*PID������ʼ��*/
void All_PID_Init(void)
{
	PID_Para_Init();
}
