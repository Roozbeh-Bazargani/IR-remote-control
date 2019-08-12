#include "IR-remote.h"

void remote(uint8_t *Rdata, TIM_HandleTypeDef htim1, uint16_t x1)
{
		if(Rdata[0]=='n')
		{
			//SendPAN(0xE0E040BF, htim1, x1);
			SendNEC(0x20DF10EF, htim1, x1);
			//SendNEC(0xFFFFFFFF);
		}
		else if(Rdata[0]=='s')
		{
			//SendNEC(0x20DF10EF, htim1, x1);
			SendSony(0x20DF, htim1, x1);
			//SendSAM(0xE0E0E01F, htim1, x1);
		}
		else if(Rdata[0]=='m')
		{
			//SendSAM(0xE0E0D02F, htim1, x1);
			SendSAM(0xB24D5FA0, htim1, x1);
			//SendNEC(0x20DF10EF, htim1, x1);
			//SendNEC(0xFFFFFFFE, htim1, x1);
		}
}
