/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：data_transfer.c
 * 描述    ：数据传输
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "data_transfer.h"
#include "usart.h"
#include "include.h"
#include "Parameter.h"
#include "rc.h"
#include "CanBusTask.h"
#include "ControlTask.h"
#include "drv_w25qxx.h"
#include "diskio.h"
#include "Ano_FcData.h"
#include "AnoParameter.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;
u8 data_to_send[50];	//发送数据缓存
u8 checkdata_to_send,checksum_to_send;

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	Usart2_Send(data_to_send, length);
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 user_cnt 	  = 3;
	static u8 rcdata_cnt 	= 10;
	static u8 motopwm_cnt	= 20;

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;		
	
	if(++cnt>200) cnt = 0;
	if(f.send_check)
	{
		f.send_check = 0;
		ANO_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
	else if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
	else if(f.send_user)
	{
		f.send_user = 0;
		ANO_DT_Send_User(1,0,0,W25QXX_TYPE,0,5,6,7,8,9,10);
//    switch((u8)pid_setup.groups.pid_18.kd)
//    {
//      case 0:
//             ANO_DT_Send_User(1,except_A.x,Roll-ctrl_angle_offset.x,ref.err_tmp.y,ref.err_lpf.y,0,
//                                0,0,0,0,400,
//                                0,0,0,0,0,
//                                0,0,0,0,0);
//               break;       
//    }
	}
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(CH[2]+1500,CH[3]+1500,CH[0]+1500,CH[1]+1500,CH[4]+1500,CH[5]+1500,CH[6]+1500,CH[7]+1500,0 +1500,0 +1500);
	}	
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,pid_setup.groups.pid_1.kp,pid_setup.groups.pid_1.ki,pid_setup.groups.pid_1.kd,
											pid_setup.groups.pid_2.kp,pid_setup.groups.pid_2.ki,pid_setup.groups.pid_2.kd,
											pid_setup.groups.pid_3.kp,pid_setup.groups.pid_3.ki,pid_setup.groups.pid_3.kd);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,pid_setup.groups.pid_4.kp,pid_setup.groups.pid_4.ki,pid_setup.groups.pid_4.kd,
											pid_setup.groups.pid_5.kp,pid_setup.groups.pid_5.ki,pid_setup.groups.pid_5.kd,
											pid_setup.groups.pid_6.kp,pid_setup.groups.pid_6.ki,pid_setup.groups.pid_6.kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		ANO_DT_Send_PID(3,pid_setup.groups.pid_7.kp,pid_setup.groups.pid_7.ki,pid_setup.groups.pid_7.kd,
											pid_setup.groups.pid_8.kp,pid_setup.groups.pid_8.ki,pid_setup.groups.pid_8.kd,
											pid_setup.groups.pid_9.kp,pid_setup.groups.pid_9.ki,pid_setup.groups.pid_9.kd);
	}
	else if(f.send_pid4)
	{
		f.send_pid4 = 0;
		ANO_DT_Send_PID(4,pid_setup.groups.pid_10.kp,pid_setup.groups.pid_10.ki,pid_setup.groups.pid_10.kd,
											pid_setup.groups.pid_11.kp,pid_setup.groups.pid_11.ki,pid_setup.groups.pid_11.kd,
											pid_setup.groups.pid_12.kp,pid_setup.groups.pid_12.ki,pid_setup.groups.pid_12.kd);
	}
	else if(f.send_pid5)
	{
		f.send_pid5 = 0;
		ANO_DT_Send_PID(5,pid_setup.groups.pid_13.kp,pid_setup.groups.pid_13.ki,pid_setup.groups.pid_13.kd,
											pid_setup.groups.pid_14.kp,pid_setup.groups.pid_14.ki,pid_setup.groups.pid_14.kd,
											pid_setup.groups.pid_15.kp,pid_setup.groups.pid_15.ki,pid_setup.groups.pid_15.kd);
	}
	else if(f.send_pid6)
	{
		f.send_pid6 = 0;
		ANO_DT_Send_PID(6,pid_setup.groups.pid_16.kp,pid_setup.groups.pid_16.ki,pid_setup.groups.pid_16.kd,
											pid_setup.groups.pid_17.kp,pid_setup.groups.pid_17.ki,pid_setup.groups.pid_17.kd,
											pid_setup.groups.pid_18.kp,pid_setup.groups.pid_18.ki,pid_setup.groups.pid_18.kd);
	}
	
	ANO_DT_Data_Receive_Anl_Task();
}


/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
static u8 RxBuffer[50],DT_data_cnt = 0,ano_dt_data_ok;
void ANO_DT_Data_Receive_Anl_Task()
{
	if(ano_dt_data_ok)
	{
		ANO_DT_Data_Receive_Anl(RxBuffer,DT_data_cnt+5);
		ano_dt_data_ok = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		DT_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+DT_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+DT_data_cnt]=data;
		ano_dt_data_ok = 1;
	}
	else
		state = 0;
}

u16 flash_save_en_cnt = 0;
u16 RX_CH[CH_NUM];

void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			;
		}
		else if(*(data_buf+4)==0X02)
			;
		else if(*(data_buf+4)==0X03)
		{
			;	
		}
		else if(*(data_buf+4)==0X04)
		{
			;
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			;
		}
		else if(*(data_buf+4)==0X20)
		{
			;
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			Para_ResetToFactorySetup();
			All_PID_Init();
			data_save();
		}
	}

	if(*(data_buf+2)==0X03)
	{
		if( NS != 1 )
		{
			Feed_Rc_Dog(2);
		}

		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        pid_setup.groups.pid_1.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.pid_1.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.pid_1.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        pid_setup.groups.pid_2.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.pid_2.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.pid_2.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        pid_setup.groups.pid_3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.pid_3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.pid_3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
			  data_save();
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
        pid_setup.groups.pid_4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.pid_4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.pid_4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        pid_setup.groups.pid_5.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.pid_5.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.pid_5.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        pid_setup.groups.pid_6.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.pid_6.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.pid_6.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				data_save();
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
        pid_setup.groups.pid_7.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.pid_7.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.pid_7.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
        pid_setup.groups.pid_8.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.pid_8.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.pid_8.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
			
        pid_setup.groups.pid_9.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.pid_9.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.pid_9.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				data_save();
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		    pid_setup.groups.pid_10.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.pid_10.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.pid_10.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
        pid_setup.groups.pid_11.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.pid_11.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.pid_11.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
        pid_setup.groups.pid_12.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.pid_12.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.pid_12.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				data_save();
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
    pid_setup.groups.pid_13.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		pid_setup.groups.pid_13.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		pid_setup.groups.pid_13.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
	
		pid_setup.groups.pid_14.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		pid_setup.groups.pid_14.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		pid_setup.groups.pid_14.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
	
		pid_setup.groups.pid_15.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		pid_setup.groups.pid_15.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		pid_setup.groups.pid_15.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		data_save();
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
    pid_setup.groups.pid_16.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		pid_setup.groups.pid_16.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		pid_setup.groups.pid_16.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
	
		pid_setup.groups.pid_17.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		pid_setup.groups.pid_17.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		pid_setup.groups.pid_17.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
	
		pid_setup.groups.pid_18.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		pid_setup.groups.pid_18.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		pid_setup.groups.pid_18.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		data_save();
	}
}

void ANO_DT_Send_Version (u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_SendString(char *str, u8 len)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xE0;
	data_to_send[_cnt++]=0;
	for(u8 i=0; i<len; i++)
		data_to_send[_cnt++] = *(str+i);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_Send_User(u8 mode,s32 d1,s32 d2,s32 d3,s32 d4,s16 d5,s16 d6,s16 d7,s16 d8,s16 d9,s16 d10)
{
	u8 _cnt=0;
	u8 sum = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=(0xf0+mode); //用户数据
	data_to_send[_cnt++]=0;
	
	
	_temp = (s16)d1;            //1
  data_to_send[_cnt++]=BYTE3(d1);
  data_to_send[_cnt++]=BYTE2(d1);
	data_to_send[_cnt++]=BYTE1(d1);
	data_to_send[_cnt++]=BYTE0(d1);

	_temp = (s16)d2;            //2
	data_to_send[_cnt++]=BYTE3(d2);
  data_to_send[_cnt++]=BYTE2(d2);
	data_to_send[_cnt++]=BYTE1(d2);
	data_to_send[_cnt++]=BYTE0(d2);
	
	_temp = (s16)d3;            //3
	data_to_send[_cnt++]=BYTE3(d3);
  data_to_send[_cnt++]=BYTE2(d3);
	data_to_send[_cnt++]=BYTE1(d3);
	data_to_send[_cnt++]=BYTE0(d3);
	
	_temp = (s16)d4;            //4
	data_to_send[_cnt++]=BYTE3(d4);
  data_to_send[_cnt++]=BYTE2(d4);
	data_to_send[_cnt++]=BYTE1(d4);
	data_to_send[_cnt++]=BYTE0(d4);
	
  _temp = (s16)d5;               //5
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = (s16)d6;               //6
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)d7;               //7
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)d8;               //8
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = (s16)d9;               //9
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = (s16)d10;              //10
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
  
	data_to_send[3] = _cnt-4;
	
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
