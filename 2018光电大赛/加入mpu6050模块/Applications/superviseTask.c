#include "superviseTask.h"

Debug_Info debug_info;
void Can_Error_Process(Can_Info *can_info);

uint32_t lost_err = 0xFFFFFFFF;     //ÿһλ����һ������
