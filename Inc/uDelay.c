#include "uDelay.h"


void uDelay(const uint16_t d)
{
	TIM3->CNT = 0;
	while(TIM3->CNT < d);
}