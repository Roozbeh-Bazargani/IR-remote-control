#include "Sony.h"

void SendSony(const uint16_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendStartSony(htim1, x1);
	SendDataSony(data, htim1, x1);
}

void SendStartSony(TIM_HandleTypeDef htim1, uint16_t x1)
{
	// 9 ms High
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(StartTime1Sony);
	// 4.5 ms Low
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(StartTime2Sony);
}

void SendDataSony(const uint16_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	uint16_t mask = mask16;
	
	for(uint16_t i=0; i<16; i++)
	{
		if (data & mask)
			SendOneSony(htim1, x1);
		else 
			SendZeroSony(htim1, x1);
		mask = mask >> 1;
	}
}

void SendOneSony(TIM_HandleTypeDef htim1, uint16_t x1)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(OneHighTimeSony);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(LowTimeSony);
}

void SendZeroSony(TIM_HandleTypeDef htim1, uint16_t x1)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(ZeroHighTimeSony);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(LowTimeSony);
}