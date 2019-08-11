#include "uDelay.h"
#include "main.h"

#define StartTime1Panasonic 		3456
#define StartTime2Panasonic 		1728
#define HighTimePanasonic 			432
#define ZeroLowTimePanasonic		432
#define OneLowTimePanasonic			1296

#define mask32		0x80000000

void SendPAN(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendStartPAN(TIM_HandleTypeDef htim1, uint16_t x1);
void SendDataPAN(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendEndPAN(TIM_HandleTypeDef htim1, uint16_t x1);
void SendHighPAN(TIM_HandleTypeDef htim1, uint16_t x1);
void SendOnePAN(TIM_HandleTypeDef htim1, uint16_t x1);
void SendZeroPAN(TIM_HandleTypeDef htim1, uint16_t x1);