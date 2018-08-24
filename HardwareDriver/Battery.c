/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
Battery.c file
------------------------------------
*/

#include "Battery.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"


uint16_t Get_Adc(uint8_t ch) {
	ADC1->SQR3 &= 0XFFFFFFE0;
	ADC1->SQR3 |= ch;		  			    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!(ADC1->SR & (1 << 1))); 	   
	return (uint16_t) ADC1->DR;
}

uint16_t Get_Adc_Average(uint8_t ch, uint8_t times) {
	uint32_t temp_val = 0;
 	uint8_t t;
	for(t = 0; t < times; t++){
		temp_val += Get_Adc(ch);
	}
	return temp_val/times;
}

int Get_Temp(void) {				 
	float temperature;   
	temperature = (float) Get_Adc_Average(16, 20) * 16.11328125e-4;
	temperature = (1.43 - temperature) / 0.0043 + 25;
	temperature *= 10;
	return (int)temperature;	 
}

float GetBattVolt(void) {
	Battery.ADCVal  = Get_Adc_Average(8,10);
	Battery.voltage = Battery.ADCVal * BATT_K;
	return Battery.voltage;
}
