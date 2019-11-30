#include "judgement.h"

u8 Judge_data;
char data_num=0;
unsigned char Judge_Rx_Buf[100],Judge_Tx_Buf[100];

tGameInfo Pro_Information;
tRealBloodChangedData BloodData;
tRealShootData ShootData;
float realPower;

void Get_Judgement_data(void)	//根据裁判系统协议读取裁判系统数据 注意17协议有变化
{
		if(Judge_Rx_Buf[0] == 0xA5)
		{	
//				if(Verify_CRC8_Check_Sum(Judge_Rx_Buf,4) && Verify_CRC16_Check_Sum(Judge_Rx_Buf,(Judge_Rx_Buf[2]<<8|Judge_Rx_Buf[1])+8))  //CRC
//				{
						if((Judge_Rx_Buf[6]<<8|Judge_Rx_Buf[5]) == Competition_Process_Information)
						{
								memcpy(&Pro_Information,Judge_Rx_Buf + 7,sizeof(tGameInfo));
								realPower = Pro_Information.realChassisOutA*Pro_Information.realChassisOutV;						
						}
						if((Judge_Rx_Buf[6]<<8|Judge_Rx_Buf[5]) == Real_time_Blood_Information)
						{
								memcpy(&BloodData,Judge_Rx_Buf + 7,sizeof(tRealBloodChangedData));
						}
						if((Judge_Rx_Buf[6]<<8|Judge_Rx_Buf[5]) == Real_time_Shoot_Information)
						{
								memcpy(&ShootData,Judge_Rx_Buf + 7,sizeof(ShootData));
						}
//				}
		}
}
