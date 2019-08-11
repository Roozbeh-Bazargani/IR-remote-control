#include "Panasonic.h"

void SendHighPAN(TIM_HandleTypeDef htim1, uint16_t x1)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(HighTimePanasonic);
	
}

void SendOnePAN(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHighPAN(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(OneLowTimePanasonic);
}

void SendZeroPAN(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHighPAN(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(ZeroLowTimePanasonic);
}

void SendStartPAN(TIM_HandleTypeDef htim1, uint16_t x1)
{
	// 9 ms High
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(StartTime1Panasonic);
	// 4.5 ms Low
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(StartTime2Panasonic);
}

void SendDataPAN(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	uint32_t mask = mask32;
	
	for(uint16_t i=0; i<32; i++)
	{
		if (data & mask)
			SendOnePAN(htim1, x1);
		else 
			SendZeroPAN(htim1, x1);
		mask = mask >> 1;
	}
	
	
}

void SendEndPAN(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHighPAN(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void SendPAN(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendStartPAN(htim1, x1);
	SendDataPAN(data, htim1, x1);
	SendEndPAN(htim1, x1);
}