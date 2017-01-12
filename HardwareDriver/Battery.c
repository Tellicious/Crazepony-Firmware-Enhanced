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
#include "ReceiveData.h"
#include "control.h"


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



// Va tolto da qui!
void BatteryCheck(void) {
	static uint8_t lowBattCount = 0;
	GetBattVolt();	

	if(flightModes.FLIGHT_ENABLED){
		if(Battery.voltage <= (BATT_LOW_VAL + 0.02)){
				warnings.LOW_BATTERY = 1;
		}
		else{
				warnings.LOW_BATTERY = 0;
		}
		
		if(Battery.voltage <= BATT_LOW_VAL){
			lowBattCount++;
			if(lowBattCount > 8){
				altCtrlMode = LANDING;
				rcData[0] = 1500; rcData[1] = 1500; rcData[2] = 1500; rcData[3] = 1500;
			}
		}
		else{
			lowBattCount = 0;
		}
		
	}
	else{
		if((Battery.voltage < BATT_THRS_VAL) && (Battery.voltage > BATT_CHG_VAL)){
			warnings.LOW_BATTERY = 1;
		}else{
			warnings.LOW_BATTERY = 0;
		}
	}
	
	if(Battery.voltage < BATT_CHG_VAL){ //on charge
		warnings.BATTERY_CHARGING = 1;
	}
	else{
		warnings.BATTERY_CHARGING = 0;
	}
}



