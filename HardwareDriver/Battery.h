#ifndef _Battery_H_
#define _Battery_H_
#include "stm32f10x.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
	
float GetBattVolt(void);
void BatteryCheck(void);

#endif
                
        



