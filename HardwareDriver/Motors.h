#ifndef _MOTO_H_
#define _MOTO_H_
#include "stdint.h"

void MotorsMix(int16_t* Motors, float Throttle, float Roll, float Pitch, float Yaw);
void MotorsPWMFlash(int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4);

#endif

