#ifndef _PID_H
#define _PID_H
#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
    float kp;	//prortional gain
    float ki;	//integral gain
    float kd;	//derivative gain
    float nd;	//derivative filter constant N: derivative in Laplace=s/(1+s/N)
    uint32_t dT;	//loop time in ms
    float kf;	//derivative filter constant
    float output;	//output value
    float intMin;	//lower integral saturation limit
    float intMax;	//upper integral saturation limit
    float eOld;	//previous step error
    float DuD;	//derivative action contribution
	float DuI;	//integral action contribution
}PID_t;

void PID_init(PID_t *PID, float kp_val, float ki_val, float kd_val, float nd_val, uint32_t dT, float sat_min, float sat_max); //dT in ms
uint8_t PID_calc(PID_t *PID, float set_point, float measure); //return 0 if integral term is saturated
float PID_getKp(PID_t *PID);
float PID_getKi (PID_t *PID);
float PID_getKd (PID_t *PID);
void PID_setKp(PID_t *PID, float kp_val);
void PID_setKi (PID_t *PID, float ki_val);
void PID_setKd (PID_t *PID, float kd_val, float nd_val);
void PID_setIntegralSaturation(PID_t *PID, float sat_min, float sat_max);
void PID_reset(PID_t *PID);

#ifdef __cplusplus
}
#endif
#endif

