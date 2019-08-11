#include "uDelay.h"
#include "main.h"

#define StartTime1SAM 		4500
#define StartTime2SAM 		4500
#define HighTimeSAM 			560
#define ZeroLowTimeSAM		560
#define OneLowTimeSAM			1690

#define mask32		0x80000000

void SendSAM(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendStartSAM(TIM_HandleTypeDef htim1, uint16_t x1);
void SendDataSAM(const uint32_t data, TIM_HandleTypeDef htim1, uint16_t x1);
void SendHighSAM(TIM_HandleTypeDef htim1, uint16_t x1);
void SendOneSAM(TIM_HandleTypeDef htim1, uint16_t x1);
void SendZeroSAM(TIM_HandleTypeDef htim1, uint16_t x1);
void SendEndSAM(TIM_HandleTypeDef htim1, uint16_t x1);