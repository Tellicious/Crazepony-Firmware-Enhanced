#ifndef _CFGPARAMS_H_
#define _CFGPARAMS_H_
#include <stdint.h>

// Constants
#define M_PI_F 		3.1415926f	//pi value
#define INS_G_VAL	9.80665f	//g value in m/s^2
#define INS_QNE_VAL	101325.0f //Mean Sea Level Pressure in Pa

// Control Parameters
#define  PR_MAX_ANGLE  30.0f // deg, da portare in rad
#define  YAW_MAX_RATE  180.0f/M_PI_F		//deg/s, da portare in rad/s

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
#define BATT_CHK_PRD 50	//ds
#define BATT_THRS_VAL  3.65	  
#define BATT_CHG_VAL    1.0
#define BATT_LOW_VAL 3.3

// Sensor reading Timeout
#define MPU_READ_TIMEOUT 1050 //us
// Timer Flags
extern uint8_t loop200HzFlag, loop100HzFlag, loop50HzFlag, loop20HzFlag, loop10HzFlag;

// NRF24 Address
extern uint8_t NRF24_rx_address[5];

// Command Values
//extern uint16_t Throttle, Roll, Pitch, Yaw;

#endif
