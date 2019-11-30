#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"

typedef struct
{
    u8 msg_id;
    u8 msg_data;
    u8 send_check;
    u8 send_version;
    u8 send_status;
    u8 send_senser;
    u8 send_senser2;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_pid5;
    u8 send_pid6;
    u8 send_rcdata;
    u8 send_offset;
    u8 send_motopwm;
    u8 send_power;
    u8 send_user;
    u8 send_speed;
    u8 send_location;

} dt_flag_t;

extern dt_flag_t f;


void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_SendString(char *str, u8 len);
void ANO_DT_Send_User(u8 mode,float d1,s32 d2,s16 d3,s16 d4,s16 d5,s16 d6,s16 d7,s16 d8,s16 d9,s16 d10);


#endif

