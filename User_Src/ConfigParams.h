#ifndef _CFGPARAMS_H_
#define _CFGPARAMS_H_
#include <stdint.h>

// Constants
#define M_PI_F 		3.1415926f	//pi value
#define INS_G_VAL	9.80665f	//g value in m/s^2
#define INS_QNE_VAL	101325.0f //Mean Sea Level Pressure in Pa
#define TORAD	0.01745329252f // from degrees to radians
#define TODEG 57.295779513f //from radians to degrees

// Control Parameters
#define VS_MAX 1.0f //m/s
#define PR_MAX_ANGLE  30.0f // deg, da portare in rad
#define YAW_MAX_RATE  180.0f/M_PI_F		//deg/s, da portare in rad/s
#define TOL_SPEED 0.3f //m/s, take-off and landing speed
#define TOFF_HEIGHT 1.0f //m, take-off height
#define GROUND_HEIGHT 0.1f //m, height below which the quad is considered to be landed

// PID Coefficients
#define PR_RATE_KP	40.1f
#define PR_RATE_KI	28.65f
#define PR_RATE_KD	1.72f
#define PR_RATE_INT_SAT 300.0f

#define YAW_RATE_KP	1145.92f
#define YAW_RATE_KI	0.0f
#define YAW_RATE_KD	0.0f
#define YAW_RATE_INT_SAT 300.0f

#define VS_KP	0.1f
#define VS_KI	0.02f
#define VS_KD	0.0f
#define VS_INT_SAT 300.0f

#define PR_ANGLE_KP	200.53f
#define PR_ANGLE_KI	0.0f
#define PR_ANGLE_KD	0.0f
#define PR_ANGLE_INT_SAT 300.0f

#define ALTITUDE_KP	1.0f
#define ALTITUDE_KI	0.0f
#define ALTITUDE_KD	0.0f
#define ALTITUDE_INT_SAT 300.0f

#define PID_KD_COEFF 100.0f;
#define PID_INNER_DT 10; //ms
#define PID_OUTER_DT 20; //ms

// Clock defines
#define RCC_CLOCK_HSE
//#define RCC_CLOCK_HSI

// UART1 defines
#define UART1_BAUDRATE 115200
#define UART_DEBUG
#define UART1_USE_INTERRUPTS
//#define UART1_NO_INTERRUPTS
#define UART1_STDIO_FUN

// TIM period (TIM3 freq: 10kHz, TIM4 freq: 1MHz)
#define TIMER3_Period		20000
#define TIMER4_Period  	1000

// Battery constant
#define BATT_K 16.35156879e-4f //ratio between battery voltage and ADC reading
#define BATT_CHK_PRD 30	//ds
#define BATT_THRS_VAL  3.65	  
#define BATT_CHG_VAL    1.0
#define BATT_LOW_VAL 3.3

// Limits
#define TILT_LIMIT 1.05f //rad
#define ALT_LIMIT 3.5f //m
#define CRASH_LIMIT 29.0f //m/s^2

// Sensor reading Timeout
#define MPU_READ_TIMEOUT 1500 //us

//Gyro Calibration measures
#define GYRO_CAL_MEASURES 500

// Timer Flags
extern uint8_t loop200HzFlag, loop100HzFlag, loop50HzFlag, loop20HzFlag, loop10HzFlag;

// NRF24 Address
extern uint8_t NRF24_rx_address[5];

// Command Values
//extern uint16_t Throttle, Roll, Pitch, Yaw;

#endif
