#ifndef _COMM_APP_H
#define _COMM_APP_H
#include "stdint.h"

enum {DISARMED=0,REQ_ARM,ARMED,REQ_DISARM};

#define MSP_ARM_IT		5
#define MSP_DISARM_IT	6
#define MSP_SET_4CON	7
#define MSP_ACC_CALI	205

#define APP_YAW_DB	 70 //dead band 
#define APP_PR_DB		 50


extern uint16_t rcData[4];
extern uint32_t lastGetRCTime;
extern uint8_t armState;

void RCDataProcess(void);

#endif

