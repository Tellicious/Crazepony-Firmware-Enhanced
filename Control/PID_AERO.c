#include "PID_AERO.h"
#define constrain(in, inf, sup) (in < inf ? inf : (in > sup ? sup : in))

PID_t PID_altitude;
PID_t PID_roll;
PID_t PID_pitch;
PID_t PID_yaw;
PID_t PID_yawRate;


void PID_init(PID_t *PID, float kp_val, float ki_val, float kd_val, float nd_val, uint32_t dT, float sat_min, float sat_max){
	float dT_sec;
	dT_sec = (float) dT * 1e-3;
	PID->dT = dT;
	PID->kp = kp_val;
	PID->ki = 0.5 * ki_val * dT_sec;
	PID->nd = nd_val;
	PID->kd = (2 * kd_val * nd_val) / (2 + nd_val * dT_sec);
	PID->kf = (2 - nd_val * dT_sec) / (2 + nd_val * dT_sec);
    PID_setIntegralSaturation(PID, sat_min, sat_max);
    PID->eOld = 0;
    PID->DuD = 0;
    PID->DuI = 0;
}

uint8_t PID_calc(PID_t *PID, float set_point, float measure){
	float e;
	e = set_point - measure;
	PID->DuI += PID->ki * (e + PID->eOld);
    PID->DuI = constrain(PID->DuI, PID->intMin, PID->intMax);
	PID->DuD = PID->kf * PID->DuD + PID->kd * (e - PID->eOld);
	PID->output = PID->kp * e + PID->DuI + PID->DuD;
    PID->eOld = e;
    if ((PID->DuI == PID->intMin) || (PID->DuI == PID->intMax)){
        return 0;
    }
    return 1;
}

float PID_getKp(PID_t *PID){
	return PID->kp;
}

float PID_getKi (PID_t *PID){
	return (2 * PID->ki * 1e3 / PID->dT);
}

float PID_getKd (PID_t *PID){
	return (0.5 * (2 + PID->nd * PID->dT * 1e-3) * PID->kd / PID->nd);
}

void PID_setKp(PID_t *PID, float kp_val){
	PID->kp = kp_val;;
}

void PID_setKi (PID_t *PID, float ki_val){
	PID->ki = 0.5 * ki_val * PID->dT * 1e-3;
}

void PID_setKd (PID_t *PID, float kd_val, float nd_val){
	PID->nd = nd_val;
	PID->kd = (2 * kd_val * nd_val) / (2 + nd_val * PID->dT * 1e-3);
	PID->kf = (2 - nd_val * PID->dT * 1e-3) / (2 + nd_val * PID->dT * 1e-3);
}

void PID_setIntegralSaturation(PID_t *PID, float sat_min, float sat_max){
	PID->intMin = sat_min;
	PID->intMax = sat_max;
}


void PID_reset(PID_t *PID){
	PID->eOld = 0;
    PID->DuD = 0;
    PID->DuI = 0;
}

