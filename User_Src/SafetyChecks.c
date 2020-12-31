#include "SafetyChecks.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
#include "stm32f10x_it.h"
#include "Battery.h"

uint32_t lastGetRCTime;

void BatteryCheck(void){
	static uint8_t lowBattCount = 0;
	GetBattVolt();	
	if(flightModes.FLIGHT_ENABLED){
		if(Battery.voltage <= BATT_LOW_VAL){
			lowBattCount++;
			if(lowBattCount > 5){
				warnings.EMPTY_BATTERY = 1;
			}
		}
		else if(Battery.voltage <= (BATT_LOW_VAL + 0.02)){
			warnings.LOW_BATTERY = 1;
			lowBattCount = 0;
		}
		else{
			warnings.EMPTY_BATTERY = 0;
			warnings.LOW_BATTERY = 0;
			lowBattCount = 0;
		}
	}
	else{
		if((Battery.voltage < BATT_THRS_VAL) && (Battery.voltage > BATT_CHG_VAL)){
			warnings.EMPTY_BATTERY = 1;
			warnings.LOW_BATTERY = 1;
		}
		else if (Battery.voltage < BATT_CHG_VAL){ //on charge
			warnings.BATTERY_CHARGING = 1;
		}
		else{
			warnings.EMPTY_BATTERY = 0;
			warnings.LOW_BATTERY = 0;
			warnings.BATTERY_CHARGING = 0;
		}
	}
}

void RCCheck(void){
	if (((millis()) - lastGetRCTime) > 2000){
		warnings.LOST_RC = 1;
	}
	return;
}

void RCCheckRst(void){
	lastGetRCTime = millis();
	return;
}

void TiltCheck(void){
	if ((AHRSData.roll > TILT_LIMIT) || (AHRSData.pitch > TILT_LIMIT)){
		warnings.TILT_EXCEEDED = 1;
	}
	return;
}

void AltitudeCheck(void){
	if (NAVData.altitude > ALT_LIMIT){
		warnings.MAX_ALTITUDE = 1;
	}
	else{
		warnings.MAX_ALTITUDE = 0;
	}
	return;
}
