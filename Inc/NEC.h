#include "uDelay.h"
#include "main.h"

#define StartTime1NEC 	9000
#define StartTime2NEC 	4500
#define HighTimeNEC 		560 //560
#define ZeroLowTimeNEC	470 //560
#define OneLowTimeNEC		1690 //1690

#define mask32		0x80000000

void SendNEC(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendStartNEC(TIM_HandleTypeDef htim1, uint16_t x1);
void SendDataNEC(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendEndNEC(TIM_HandleTypeDef htim1, uint16_t x1);
void SendHigh(TIM_HandleTypeDef htim1, uint16_t x1);
void SendOne(TIM_HandleTypeDef htim1, uint16_t x1);
void SendZero(TIM_HandleTypeDef htim1, uint16_t x1);
