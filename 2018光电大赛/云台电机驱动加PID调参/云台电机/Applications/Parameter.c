#include "Parameter.h"
#include "string.h"
#include "ff.h"
#include "ControlTask.h"
#include "include.h"

#define SENSOR_SETUP_FILE      "sensor.bin"
#define PID_SETUP_FILE         "pid.bin"

u8 flash_init_error;
sensor_setup_t sensor_setup;
pid_setup_t pid_setup;


/* �ļ���ض��� */
static FATFS fs;
static 	FIL file;
static 	DIR DirInf;
	
static int32_t Para_ReadSettingFromFile(void)
{
	FRESULT  result;
	uint32_t bw;

 	/* �����ļ�ϵͳ */
  //f_mountΪ�����ļ�ϵͳ������fsΪָ���ļ�ϵͳ��ָ�� 0����ʾSPI Flash �ĸ�Ŀ¼ ������������ʾ���ٹ���
	result = f_mount(&fs, "0:", 1);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		/* ������ز��ɹ������и�ʽ�� */
		result = f_mkfs("0:",0,0);
		if (result != FR_OK)
		{
			 return -1;     //flash�����⣬�޷���ʽ��
		}
		else
		{
			/* ���½��й��� */
			result = f_mount(&fs, "0:", 0);			/* Mount a logical drive */
	   if (result != FR_OK)
      {
			 	/* ж���ļ�ϵͳ */
	      f_mount(NULL, "0:", 0);
			 return -2 ;
			}
		}
	}

	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "/"); 
	if (result != FR_OK)
	{
		/* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
		return -3;
	}

	/* ���ļ� */
	result = f_open(&file, SENSOR_SETUP_FILE, FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
	  /* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
   /* �ļ������� */
		return -4;
	}

	/* ��ȡSensor�����ļ� */
	result = f_read(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);
	if (bw > 0)
	{
		/* �ر��ļ�*/
	 f_close(&file);
		/* ���ļ� */
	 result = f_open(&file, PID_SETUP_FILE, FA_OPEN_EXISTING | FA_READ);
	  if (result !=  FR_OK)
	 {
		/* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
		return -4;
	 }
		/* ��ȡPID�����ļ� */
	 result = f_read(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);
    if(bw > 0)
		{
		 /* �ر��ļ�*/
	   f_close(&file);
		 	/* ж���ļ�ϵͳ */
	   f_mount(NULL, "0:", 0);
			return 1;
		}else
		{
		 /* �ر��ļ�*/
	   f_close(&file);
		 	/* ж���ļ�ϵͳ */
	    f_mount(NULL, "0:", 0);
			return -4;
		}
	}else
  {
	 /* �ر��ļ�*/
	 f_close(&file);
	 	/* ж���ļ�ϵͳ */
	 f_mount(NULL, "0:", 0);
	 return -5;
	}

}

static int32_t Para_WriteSettingToFile(void)
{
	FRESULT result;
	uint32_t bw;

 	/* �����ļ�ϵͳ */
	result = f_mount(&fs, "0:", 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		/* ������ز��ɹ������и�ʽ�� */
		result = f_mkfs("0:",0,0);
		if (result != FR_OK)
		{
			 return -1;     //flash�����⣬�޷���ʽ��
		}
		else
		{
			/* ���½��й��� */
			result = f_mount(&fs, "0:", 0);			/* Mount a logical drive */
	   if (result != FR_OK)
      {
			 	/* ж���ļ�ϵͳ */
	      f_mount(NULL, "0:", 0);
			 return -2 ;
			}
		}
	}

	/* �򿪸��ļ��� */
	result = f_opendir(&DirInf, "/"); 
	if (result != FR_OK)
	{
		/* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
		return -3;
	}

	/* ���ļ� */
	result = f_open(&file, SENSOR_SETUP_FILE, FA_CREATE_ALWAYS | FA_WRITE);
	if (result !=  FR_OK)
	{
	  /* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
		return -4;
	}

	/* д��Sensor�����ļ� */
	result = f_write(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);
	if (result == FR_OK)
	{
		/* �ر��ļ�*/
	 f_close(&file);
		/* ���ļ� */
	 result = f_open(&file, PID_SETUP_FILE, FA_CREATE_ALWAYS | FA_WRITE);
	  if (result !=  FR_OK)
	 {
		/* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
		return -4;
	 }
		/* д��PID�����ļ� */
	 result = f_write(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);
    if(result == FR_OK)
		{
	 		/* �ر��ļ�*/
	    f_close(&file);
		 	/* ж���ļ�ϵͳ */
	   f_mount(NULL, "0:", 0);
			return 1;
		}else
		{
		  /* �ر��ļ�*/
	   f_close(&file);
		 	/* ж���ļ�ϵͳ */
	    f_mount(NULL, "0:", 0);
			return -4;
		}
	}else
  {
	  /* �ر��ļ�*/
	  f_close(&file);
		/* ж���ļ�ϵͳ */
	  f_mount(NULL, "0:", 0);
	 return -5;
	}

}

void Para_ResetToFactorySetup(void)
{
	pid_setup.groups.pid_1.kp = 1.0f;
	pid_setup.groups.pid_1.ki = 1.0f;
	pid_setup.groups.pid_1.kd = 1.0f;
	
	pid_setup.groups.pid_2.kp = 1.0f;
	pid_setup.groups.pid_2.ki = 1.0f;	
	pid_setup.groups.pid_2.kd = 1.0f;	
	
	
	pid_setup.groups.pid_3.kp = 1.0f;
	pid_setup.groups.pid_3.ki = 1.0f;	
	pid_setup.groups.pid_3.kd = 1.0f;	
	
	pid_setup.groups.pid_4.kp = 1.0f;
	pid_setup.groups.pid_4.ki = 1.0f;	
	pid_setup.groups.pid_4.kd = 1.0f;	
	
	pid_setup.groups.pid_5.kp = 1.0f;
	pid_setup.groups.pid_5.ki = 1.0f;	
	pid_setup.groups.pid_5.kd = 1.0f;	
	
	pid_setup.groups.pid_6.kp = 1.0f;
	pid_setup.groups.pid_6.ki = 1.0f;	
	pid_setup.groups.pid_6.kd = 1.0f;	
	
	pid_setup.groups.pid_7.kp = 1.0f;
	pid_setup.groups.pid_7.ki = 1.0f;	
	pid_setup.groups.pid_7.kd = 1.0f;	
	
	pid_setup.groups.pid_8.kp = 1.0f;
	pid_setup.groups.pid_8.ki = 1.0f;	
	pid_setup.groups.pid_8.kd = 1.0f;	
	
	pid_setup.groups.pid_9.kp = 1.0f;
	pid_setup.groups.pid_9.ki = 1.0f;	
	pid_setup.groups.pid_9.kd = 1.0f;	
	
	pid_setup.groups.pid_9.kp = 1.0f;
	pid_setup.groups.pid_9.ki = 1.0f;
	pid_setup.groups.pid_9.kd = 1.0f;
	
	pid_setup.groups.pid_10.kp = 1.0f;
	pid_setup.groups.pid_10.ki = 1.0f;
	pid_setup.groups.pid_10.kd = 1.0;
	
	pid_setup.groups.pid_11.kp = 1.0f;
	pid_setup.groups.pid_11.ki = 1.0f;
	pid_setup.groups.pid_11.kd = 1.0f;
	
	pid_setup.groups.pid_12.kp = 1.0f;
	pid_setup.groups.pid_12.ki = 1.0f;
	pid_setup.groups.pid_12.kd = 1.0;
	
	pid_setup.groups.pid_13.kp = 1.0f;
	pid_setup.groups.pid_13.ki = 1.0f;
	pid_setup.groups.pid_13.kd = 1.0f;
	
	pid_setup.groups.pid_14.kp = 1.0f;
	pid_setup.groups.pid_14.ki = 1.0f;
	pid_setup.groups.pid_14.kd = 1.0f;
	
	pid_setup.groups.pid_15.kp = 1.0f;
	pid_setup.groups.pid_15.ki = 1.0f;
	pid_setup.groups.pid_15.kd = 1.0f;
	
	pid_setup.groups.pid_16.kp = 1.0f;
	pid_setup.groups.pid_16.ki = 1.0f;
	pid_setup.groups.pid_16.kd = 1.0;
	
	pid_setup.groups.pid_17.kp = 1.0f;
	pid_setup.groups.pid_17.ki = 1.0f;
	pid_setup.groups.pid_17.kd = 1.0f;
	
	pid_setup.groups.pid_18.kp = 1.0f;
	pid_setup.groups.pid_18.ki = 1.0f;
	pid_setup.groups.pid_18.kd = 1.0f;
}

void PID_Para_Init()
{
	GMYawControl_ParaInit();
//	Ctrl_Para_Init();
//	h_pid_init();

}

void Para_Init()
{
	int32_t result = Para_ReadSettingFromFile();
  if(result < 0)
  {
	 Para_ResetToFactorySetup();
	 flash_init_error = 1;
	}
	//Param_SetSettingToFC();
	
	PID_Para_Init();
}

void Param_SavePID(void)
{
// memcpy(&pid_setup.groups.ctrl1.roll,&ctrl_1.PID[PIDROLL],sizeof(pid_t));
// memcpy(&pid_setup.groups.ctrl1.pitch,&ctrl_1.PID[PIDPITCH],sizeof(pid_t));
// memcpy(&pid_setup.groups.ctrl1.yaw,&ctrl_1.PID[PIDYAW],sizeof(pid_t));
//  
// memcpy(&pid_setup.groups.ctrl2.roll,&ctrl_2.PID[PIDROLL],sizeof(pid_t));
// memcpy(&pid_setup.groups.ctrl2.pitch,&ctrl_2.PID[PIDPITCH],sizeof(pid_t));
// memcpy(&pid_setup.groups.ctrl2.yaw,&ctrl_2.PID[PIDYAW],sizeof(pid_t));
 Para_WriteSettingToFile();
}

extern u16 flash_save_en_cnt;

void Parameter_Save()
{
  if( flash_save_en_cnt !=0 )
	{
		flash_save_en_cnt++;
	}

	if( flash_save_en_cnt > 60 ) // 20 *60 = 1200ms
	{
		flash_save_en_cnt = 0;
		if( 1 )
		{
			Param_SavePID();
		}
	}
}


