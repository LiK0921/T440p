#include "Ano_FcData.h"
#include "AnoParameter.h"

void data_save(void)
{
	para_sta.save_trig = 1;
}

void Para_Data_Init()
{
	Ano_Parame_Read();
}
