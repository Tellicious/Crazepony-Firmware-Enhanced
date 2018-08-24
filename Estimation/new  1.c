#include "Altitude_KF.h"
typedef struct{
	float h, a, v;
	float m_hh, m_ha, m_ah, m_aa, m_vh, m_va; //gain matrix
	float T, T2_2; //step time
} alt_KF_t;
alt_KF_t alt_KF;


alt_KF.T = xxxxxxxx;
alt_KF.T2_2 = (alt_KF.T * alt_KF.T) / 2;

void alt_KF_prediction (void){
	alt_KF.h += alt_KF.T2_2 * alt_KF.a + alt_KF.T * alt_KF.v;
	alt_KF.v += alt_KF.T * alt_KF.a;
	return;
}

void alt_KF_update_baro (float baro_press){
	float alt_delta = xxxx - alt_KF.h;
	alt_KF.h += alt_KF.m_hh * alt_delta;
	alt_KF.a += alt_KF.m_ah * alt_delta;
	alt_KF.v += alt_KF.m_vh * alt_delta;
	alt_KF_update_INS();
	return;
}

void alt_KF_update_accel (float accel_D){
	float accel_delta = -accel_D - alt_KF.a + INS_G_VAL; //minus sign because accel value is pointing down
	alt_KF.h += alt_KF.m_ha * accel_delta;
	alt_KF.a += alt_KF.m_aa * accel_delta;
	alt_KF.v += alt_KF.m_va * accel_delta;
	alt_KF_update_INS();
	return;
}

void alt_KF_update_INS(void){
	INS.altitude = alt_KF.h;
	INS.climb_rate = alt_KF.v;
	return;
}