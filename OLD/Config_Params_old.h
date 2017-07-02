#ifndef _CFG_PARAMS_H_
#define _CFG_PARAMS_H_
#include <stdint.h>

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


// Timer Flags
extern uint8_t loop200HzFlag, loop100HzFlag, loop50HzFlag, loop20HzFlag, loop10HzFlag;

// NRF24 Address
extern uint8_t NRF24_rx_address[5];

// Command Values
//extern uint16_t Throttle, Roll, Pitch, Yaw;

#endif
