#include "Control.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
#include "PID_AERO.h"
#include "Motors.h"

//PID_t PID_altitude;
PID_t PID_vertSpeed;
PID_t PID_roll;
PID_t PID_rollRate;
PID_t PID_pitch;
PID_t PID_pitchRate;
PID_t PID_yawRate;
int16_t Motors[4]={0};


void CtrlPIDInit(void){
	//PID_init(&PID_altitude, ALTITUDE_KP, ALTITUDE_KI, ALTITUDE_KD, PID_KD_COEFF, PID_OUTER_DT, -ALTITUDE_INT_SAT, ALTITUDE_INT_SAT);
	PID_init(&PID_vertSpeed, VS_KP, VS_KI, VS_KD, PID_KD_COEFF, PID_OUTER_DT, -VS_INT_SAT, VS_INT_SAT);
	PID_init(&PID_roll, PR_ANGLE_KP, PR_ANGLE_KI, PR_ANGLE_KD, PID_KD_COEFF, PID_OUTER_DT, -PR_ANGLE_INT_SAT, PR_ANGLE_INT_SAT);
	PID_init(&PID_rollRate, PR_RATE_KP, PR_RATE_KI, PR_RATE_KD, PID_KD_COEFF, PID_INNER_DT, -PR_RATE_INT_SAT, PR_RATE_INT_SAT);
	PID_init(&PID_pitch, PR_ANGLE_KP, PR_ANGLE_KI, PR_ANGLE_KD, PID_KD_COEFF, PID_OUTER_DT, -PR_ANGLE_INT_SAT, PR_ANGLE_INT_SAT);
	PID_init(&PID_pitchRate, PR_RATE_KP, PR_RATE_KI, PR_RATE_KD, PID_KD_COEFF, PID_INNER_DT, -PR_RATE_INT_SAT, PR_RATE_INT_SAT);
	PID_init(&PID_yawRate, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, PID_KD_COEFF, PID_INNER_DT, -YAW_RATE_INT_SAT, YAW_RATE_INT_SAT);
	return;
}

void CtrlAttiRate(void){
	PID_calc(&PID_rollRate, PID_roll.output, INSData.cal_gyro[0]);
	PID_calc(&PID_pitchRate, PID_pitch.output, INSData.cal_gyro[1]);
	PID_calc(&PID_yawRate, APData.yawRate, INSData.cal_gyro[2]);
	if((!flightModes.IN_FLIGHT) | (!flightModes.FLIGHT_ENABLED)){
		PID_reset(&PID_rollRate);
		PID_reset(&PID_pitchRate);
		PID_reset(&PID_yawRate);
	} 
	return;
}

void CtrlAttiAngle(void){
	PID_calc(&PID_roll, APData.roll, AHRSData.roll);
	PID_calc(&PID_pitch, APData.pitch, AHRSData.pitch);
	if((!flightModes.IN_FLIGHT) | (!flightModes.FLIGHT_ENABLED)){
		PID_reset(&PID_roll);
		PID_reset(&PID_pitch);
	}
	return;
}

void CtrlAlti(void){
	PID_calc(&PID_vertSpeed, APData.vs, NAVData.rateOfClimb);
	if((!flightModes.IN_FLIGHT) | (!flightModes.FLIGHT_ENABLED)){
		PID_reset(&PID_roll);
	}
	return;
}


void CtrlMotors(void){ //Da sistemare
	MotorsMix(Motors, 0, PID_rollRate.output, PID_pitchRate.output, PID_yawRate.output);
   	if(flightModes.FLIGHT_ENABLED && flightModes.IN_FLIGHT){
		MotorsPWMFlash(Motors[0],Motors[1],Motors[2],Motors[3]);
	}
	else {                  
		MotorsPWMFlash(0,0,0,0);
	}
	return;
}
