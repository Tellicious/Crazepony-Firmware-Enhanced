#include "Autopilot.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
#include "SafetyChecks.h"
#include "LED.h"
#include "MPU6050.h"
#include "IMU.h"
#include "math.h"
#include "delay.h"
#define SCALE(x) ((x - 1500) * 2e-3f)

void APChecks(void){
	if ((RCData.command == REQ_ARM) && (warnings.ALL == 0)){
		flightModes.ALL = 0;
		flightModes.FLIGHT_ENABLED = 1;
		RCData.command = 0;
	}
	else if (RCData.command == REQ_DISARM){
		flightModes.ALL = 0;
		RCData.command = 0;
	}
	// comandi per TO e L
	
	
	if (warnings.CRASH | warnings.TILT_EXCEEDED){
		flightModes.ALL = 0;
	}
	else if (warnings.LOST_RC){
		flightModes.WINGS_LVL = 1;
		flightModes.LANDING = 1;
	}
	else if (warnings.EMPTY_BATTERY){
		flightModes.LANDING = 1;
	}
	return;	
}

void APCommand(void){ //50Hz
	static uint16_t landedCnt = 0;
	//Vertical Modes
	if (flightModes.LANDING && flightModes.FLIGHT_ENABLED){
		APData.vs = - TOL_SPEED;
		if (absf(NAVData.rateOfClimb) < 0.01){
			landedCnt++;
		}
		else{
			landedCnt = 0;
		}
		if ((absf(NAVData.rateOfClimb) < 0.01) && (landedCnt > 100)){
			APData.vs = 0;
			flightModes.ALL = 0;
			landedCnt = 0;
		}
	}
	else if (flightModes.TAKEOFF && flightModes.FLIGHT_ENABLED){
		flightModes.IN_FLIGHT = 1;
		if (NAVData.altitude < TOFF_HEIGHT) {
			APData.vs = TOL_SPEED;
		}
		else{
			APData.vs = VS_MAX * SCALE(RCData.throttle);
			flightModes.TAKEOFF = 0;
		}
	}
	else if (warnings.MAX_ALTITUDE){
		float vs_cmd = VS_MAX * SCALE(RCData.throttle); 
		APData.vs = (vs_cmd < 0)? vs_cmd : 0;
	}
	else{
		APData.vs = VS_MAX * SCALE(RCData.throttle);
	}
	
	//Attitude Modes
	if (flightModes.WINGS_LVL && flightModes.FLIGHT_ENABLED){
		APData.roll = 0;
		APData.pitch = 0;
		APData.yawRate = 0;
	}
	else{
		APData.roll = PR_MAX_ANGLE * SCALE(RCData.roll);
		APData.pitch = PR_MAX_ANGLE * SCALE(RCData.pitch);
		APData.yawRate = YAW_MAX_RATE * SCALE(RCData.yaw);
	}
}

void APPreFlightChecks(void){
	//Set all warning
	warnings.EMPTY_BATTERY = 1;
	warnings.LOW_BATTERY = 1;
	warnings.BATTERY_CHARGING = 1;
	warnings.IMU_NOT_CALIBRATED = 1;
	warnings.LOST_RC = 1;
	warnings.IMU_ST_NOT_PASSED = 0; //IMU Self-Test has to be requested
	warnings.IMU_NOT_RESPONDING = 1;
	warnings.CRASH = 0;	//A crash cannot occur at start-up
	warnings.TILT_EXCEEDED = 1;
	warnings.MAX_ALTITUDE = 0; //The quad is still on ground
	//Clean flightModes
	flightModes.ALL = 0;
	//Check Battery: if OK, first 3 warnings are cleared
	BatteryCheck();
	//Check if MPU responds
	if (MPU6050_read_STATUS(1e6)){
		warnings.IMU_NOT_RESPONDING = 0;
	}
	//Check if RC is connected
	uint32_t now = millis();
    while ((millis() - now) < 1000L){
		if (NRF24_available()){
			NRF24_flushRXBuffer();
			warnings.LOST_RC = 0;
			break;
		}
		delay_ms(20);
		if ((int32_t) (millis() - now) < 0){
			now = 0L;
		}
	}	
	//Perform a gyro calibration: if successful, it resets IMU_NOT_CALIBRATED
	calibrateGyro(GYRO_CAL_MEASURES);
	//Check if tilt is acceptable
	if((absf(raw_accel[0]) < 5.0f) && (absf(raw_accel[1]) < 5.0f)){
		warnings.TILT_EXCEEDED = 0;
	}
	//Proceed only if every warning has been cleared
	if (warnings.ALL){
		while(1){
			LEDBlinkFR();
		}
	}
	// Power On LEDs sequence
	LEDPowerOn();
	return;
}