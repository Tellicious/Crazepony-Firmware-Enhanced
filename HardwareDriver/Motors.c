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
*/
#include "stm32f10x.h"
#include "Motors.h"
#define MOTORS_MAX_PWM 999
#define MOTORS_MIN_PWM 0
#define MOTOR_PWM_CONSTRAIN(x) ((x > MOTORS_MAX_PWM)?(MOTORS_MAX_PWM):((x < MOTORS_MIN_PWM)? MOTORS_MIN_PWM : x));

void MotorsMix(int16_t* Motors, float Throttle, float Roll, float Pitch, float Yaw){
	Motors[0] = (int16_t)(Throttle + Pitch + Roll - Yaw);    //M1
	Motors[1] = (int16_t)(Throttle + Pitch - Roll + Yaw);    //M2	
	Motors[2] = (int16_t)(Throttle - Pitch - Roll - Yaw);    //M3  
	Motors[3] = (int16_t)(Throttle - Pitch + Roll + Yaw);    //M4 
}
/*
void MotorsMix(int16_t* Motors, float Throttle, float Yaw, float Pitch, float Roll){
omega_0 = fmax(omega_0, omega_min);
    
    U(0,0) = 4 * constrain(0.25 * U(0,0), (omega_min - omega_0), (omega_max - omega_0));
    float omega_th = omega_0 + 0.25 * U(0,0);
    
    float Du = 2 * fmin( fabs(omega_max - omega_th), fabs(omega_min - omega_th));
    U(1,0) = constrain(U(1,0), - Du, Du);
    U(2,0) = constrain(U(2,0), - Du, Du);
    
    float U_m = fmax(fabs(U(1,0)), fabs(U(2,0)));
    Du = 4 * fmin(fabs(omega_max - omega_th - 0.5 * U_m), fabs(omega_min - omega_th + 0.5 * U_m));
    U(3,0) = constrain(U(3,0), -Du , Du);
    
    MatrixXf M(4,1);
    M(0,0) = omega_th + 0.5 * U(2,0) - 0.25 * U(3,0);
    M(1,0) = omega_th - 0.5 * U(1,0) + 0.25 * U(3,0);
    M(2,0) = omega_th - 0.5 * U(2,0) - 0.25 * U(3,0);
    M(3,0) = omega_th + 0.5 * U(1,0) + 0.25 * U(3,0);
    return M;
	}
*/

void MotorsPWMFlash(int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4){		
	TIM2->CCR1 = MOTOR_PWM_CONSTRAIN(Motor1);
	TIM2->CCR2 = MOTOR_PWM_CONSTRAIN(Motor2);
	TIM2->CCR3 = MOTOR_PWM_CONSTRAIN(Motor3);
	TIM2->CCR4 = MOTOR_PWM_CONSTRAIN(Motor4);
}
