/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：rc.c
 * 描述    ：遥控器通道数据处理
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "include.h"
#include "rc.h"
#include "mymath.h"
#include "filter.h"

s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5,6,7};    //通道映射
u8 rc_lose = 0;

void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)      //通道功能映射
{
	u8 i;
	for( i = 0 ; i < CH_NUM ; i++ )
	{
		*( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
	}
}

s16 CH[CH_NUM];

float CH_Old[CH_NUM];
float CH_filter[CH_NUM];
float CH_filter_Old[CH_NUM];
float CH_filter_D[CH_NUM];
u8 NS,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];
 
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };	//摇杆最大
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };	//摇杆最小
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };  //摇杆方向
#define CH_OFFSET 500   //摇杆初始状态设置为对应油门500的值


float filter_A;

void RC_Duty( float T , u16 tmp16_CH[CH_NUM] )//遥控任务
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	static u16 Mapped_CH[CH_NUM];

	if( NS == 1 )
	{
		CH_Mapping_Fun(tmp16_CH,Mapped_CH);
	}
  //在PWM输入捕获失败的情况下
	else if( NS == 2 )
	{
		CH_Mapping_Fun(RX_CH,Mapped_CH);
	}

	
	for( i = 0;i < CH_NUM ; i++ )//检查每一个通道的数据是否正常
	{
		if( (u16)Mapped_CH[i] > 2500 || (u16)Mapped_CH[i] < 500 )
		{
			CH_Error[i]=1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if( CLR_CH_Error[i] > 200 )
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if( NS == 1 || NS == 2 )
		{
			if( CH_Error[i] ) //单通道数据错误
			{
				
			}
			else
			{
				//CH_Max_Min_Record();
				CH_TMP[i] = ( Mapped_CH[i] ); //映射拷贝数据，大约 1000~2000
				
				if( MAX_CH[i] > MIN_CH[i] )
				{
					if( !CH_DIR[i] )
					{
            //将摇杆对应的量转化为对应油门，摇杆对应范围为800（1100~1900），而油门对应范围为1000（-500~500）
						CH[i] =   LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
					else
					{
						CH[i] = - LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
				}	
				else
				{
					//fly_ready = 0;
				}
			}
			rc_lose = 0;
		}
    //除了NS为1或者2的情况都表示接受器未接收到信号
		else //未接接收机或无信号（遥控关闭或丢失信号）
		{
			rc_lose = 1;
		}
//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 		
    //CH_filter为通过滤波后得到的油门值
			filter_A = 6.28f *10 *T;
			
			if( ABS(CH_TMP[i] - CH_filter[i]) <100 )
			{
				CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;  //对通道进行滤波处理
			}
			else
			{
				CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
			}
			
			if(NS == 0) //无信号且为解锁的情况下
			{
					CH_filter[THR] = -500;
			}
// 					CH_filter[i] = Fli_Tmp;
			CH_filter_D[i] 	= ( CH_filter[i] - CH_filter_Old[i] );
			CH_filter_Old[i] = CH_filter[i];
			CH_Old[i] 		= CH[i];
	}
	//======================================================================
	//Fly_Ready(T,wz_speed);		//解锁判断
	//======================================================================
  //在正常喂狗情况下，NS_cnt会不断清零
	if(++NS_cnt>200)  // 400ms  未插信号线。
	{
		NS_cnt = 0;
		NS = 0;
	}
}

void Feed_Rc_Dog(u8 ch_mode) //400ms内必须调用一次，喂狗
{
	NS = ch_mode;
	NS_cnt = 0;
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

