#include "uDelay.h"
#include "main.h"

#define StartTime1Sony 		2200 //2400
#define StartTime2Sony 		600  //600
#define LowTimeSony 			520  //600
#define ZeroHighTimeSony	550 //600
#define OneHighTimeSony		1200

#define mask16		0x8000

void SendSony(const uint16_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendStartSony(TIM_HandleTypeDef htim1, uint16_t x1);
void SendDataSony(const uint16_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendOneSony(TIM_HandleTypeDef htim1, uint16_t x1);
void SendZeroSony(TIM_HandleTypeDef htim1, uint16_t x1);
