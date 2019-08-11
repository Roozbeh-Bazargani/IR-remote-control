#include "NEC.h"



void SendHigh(TIM_HandleTypeDef htim1, uint16_t x1)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(HighTimeNEC);
	
}

void SendOne(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHigh(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(OneLowTimeNEC);
}

void SendZero(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHigh(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(ZeroLowTimeNEC);
}

void SendStartNEC(TIM_HandleTypeDef htim1, uint16_t x1)
{
	// 9 ms High
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(StartTime1NEC);
	// 4.5 ms Low
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(StartTime2NEC);
}

void SendDataNEC(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	uint32_t mask = mask32;
	
	for(uint16_t i=0; i<32; i++)
	{
		if (data & mask)
			SendOne(htim1, x1);
		else 
			SendZero(htim1, x1);
		mask = mask >> 1;
	}
	
	
}

void SendEndNEC(TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendHigh(htim1, x1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void SendNEC(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1)
{
	SendStartNEC(htim1, x1);
	SendDataNEC(data, htim1, x1);
	SendEndNEC(htim1, x1);
}