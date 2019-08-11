#include "Samsung.h"

void SendHighSAM(TIM_HandleTypeDef htim1, uint16_t x1)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(HighTimeSAM);
	
}

void SendOneSAM(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHighSAM(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(OneLowTimeSAM);
}

void SendZeroSAM(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHighSAM(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(ZeroLowTimeSAM);
}

void SendStartSAM(TIM_HandleTypeDef htim1, uint16_t x1)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(StartTime1SAM);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(StartTime2SAM);
}

void SendDataSAM(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	uint32_t mask = mask32;
	
	for(uint16_t i=0; i<32; i++)
	{
		if (data & mask)
			SendOneSAM(htim1, x1);
		else 
			SendZeroSAM(htim1, x1);
		mask = mask >> 1;
	}
}


void SendSAM(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendStartSAM(htim1, x1);
	SendDataSAM(data, htim1, x1);
	SendEndSAM(htim1, x1);
}

void SendEndSAM(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendZeroSAM(htim1, x1);
}