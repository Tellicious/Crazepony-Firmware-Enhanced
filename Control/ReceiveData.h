#ifndef _ReceiveData_H_
#define _ReceiveData_H_
#include "stm32f10x.h"

extern uint8_t RCRawData[32]; //raw data received from NRF

#define MSP_ARM_IT		5
#define MSP_DISARM_IT	6
#define MSP_SET_4CON	7
#define MSP_ACC_CALI	205

void RCDataProcess(void);
void ReceiveDataFromRC(void);

#endif

