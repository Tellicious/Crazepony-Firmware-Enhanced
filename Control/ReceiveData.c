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
ReceiveData.c file
*/

  
#include "ReceiveData.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
#include "delay.h"
#include "stm32f10x_it.h"

#include "control.h"
#include "IMU.h"

#define REQ_ARM 0xA4
#define REQ_DISARM 0xB2
#define REQ_IMU_CAL 0xC1
#define REQ_LAND
#define REQ_TAKEOFF

#define CONSTRAIN(a,min,max) ((a<min)?(a=min):((a>max)?(a=max):(a=a)))

uint8_t RCRawData[32]; //raw data received from NRF

void RCDataProcess(void){
/*
	if ((RCData.command == REQ_ARM) && (warnings.ALL == 0)){
		flightModes.FLIGHT_ENABLED = 1;
	}
	else if (RCData.command == REQ_DISARM){
		flightModes.FLIGHT_ENABLED = 0;
		//////////////////// CHE SERVE?
		altCtrlMode=MANUAL;		//上锁后加的处理
		zIntReset=1;
		thrustZSp=0;	
		thrustZInt=estimateHoverThru();
		offLandFlag=0;
		///////////////////
	}
	RCData.command = 0;
	*/
	RCData.throttle = (float) ((uint16_t) RCRawData[5] | (RCRawData[6]<<8));
	RCData.roll = (float) ((uint16_t) RCRawData[11] | (RCRawData[12]<<8));
	RCData.pitch = (float) ((uint16_t) RCRawData[9] | (RCRawData[10]<<8));
	RCData.yaw = (float) ((uint16_t) RCRawData[7]   |  (RCRawData[8]<<8));
	
	CONSTRAIN(RCData.throttle, 1000, 2000);
	CONSTRAIN(RCData.roll, 1000, 2000);
	CONSTRAIN(RCData.pitch, 1000, 2000);
	CONSTRAIN(RCData.yaw, 1000, 2000);
	return;
}

void ReceiveDataFromRC(void) {
	if((RCRawData[0] == '$') && (RCRawData[1] == 'M') && (RCRawData[2] == '<')) {
		if (RCRawData[4] == MSP_ARM_IT){
			RCData.command = REQ_ARM;
		}
		else if (RCRawData[4] == MSP_DISARM_IT){
			RCData.command = REQ_DISARM;
		}
		else if (RCRawData[4] == MSP_ACC_CALI){
			imuCaliFlag = 1;
			RCData.command = REQ_IMU_CAL;
		}
	}	
	NRF24_flushRXBuffer();
}










/*void RCDataProcess(void){

	if(LANDING == altCtrlMode){
		rcData[THROTTLE] = 1500;
		rcData[YAW] = 1500;
		rcData[PITCH] = 1500;
		rcData[ROLL] = 1500;
	}
	
	CONSTRAIN(rcData[THROTTLE],1000,2000);
	CONSTRAIN(rcData[YAW],1000,2000);
	CONSTRAIN(rcData[PITCH],1000,2000);
	CONSTRAIN(rcData[ROLL],1000,2000);
 
	RCData.throttle=rcData[THROTTLE]-1000;
	RCData.yaw= YAW_MAX_RATE * dbScaleLinear((rcData[YAW] - 1500),500,APP_YAW_DB);
	RCData.pitch= PR_MAX_ANGLE * dbScaleLinear((rcData[PITCH] - 1500),500,APP_PR_DB);
	RCData.roll= PR_MAX_ANGLE * dbScaleLinear((rcData[ROLL] - 1500),500,APP_PR_DB);
	
	switch(RCData.command)
	{
		case REQ_ARM:
			if(IMUCheck() && !warnings.LOW_BATTERY){	
				RCData.command=ARMED;
				flightModes.FLIGHT_ENABLED=0xA5;
			}else{
				flightModes.FLIGHT_ENABLED=0;
				RCData.command=DISARMED;
			}
		break;
		case REQ_DISARM:
			flightModes.FLIGHT_ENABLED=0;
			altCtrlMode=MANUAL;		//上锁后加的处理
			zIntReset=1;
			thrustZSp=0;	
			thrustZInt=estimateHoverThru();
			offLandFlag=0;
			
			RCData.command=DISARMED;
		break;
		default:
			break;
			
	}
	

}*/

/*void ReceiveDataFromRC(void) {
	if((RCRawData[0] == '$') && (RCRawData[1] == 'M') && (RCRawData[2] == '<')) {
		 switch(RCRawData[4]){
			 case MSP_SET_4CON:
					rcData[THROTTLE] = RCRawData[5] | (RCRawData[6]<<8);
					rcData[YAW] = RCRawData[7]   |  (RCRawData[8]<<8);
					rcData[PITCH] = RCRawData[9] | (RCRawData[10]<<8);
					rcData[ROLL] = RCRawData[11] | (RCRawData[12]<<8);
					break;
			  case MSP_ARM_IT:
						RCData.command = REQ_ARM;
				break;
			 case MSP_DISARM_IT:
					RCData.command = REQ_DISARM;
			 break;
			 case MSP_ACC_CALI:
					imuCaliFlag = 1;
			 break;
		 }
	}	
	lastGetRCTime = millis();	//ms
	NRF24_flushRXBuffer();
}
*/

/*****NRF24L01 match *****/
/*static uint8_t sta;
extern u8  RX_ADDRESS[RX_ADR_WIDTH];		
extern void SaveParamsToEEPROM(void);
u8 NRFMatched = 0;

void NRFmatching(void){
	static uint32_t nTs,nT;
	static uint32_t writeOvertime = 2 * 1000000;// unit :us
	
	LED3_ON;   //led3 always on when 2.4G matching
	nTs = micros();
	
  do  
	{   
		  NRFMatched = 0;
		  nT = micros() - nTs;
		  
		  if(nT >= writeOvertime){
				RX_ADDRESS[4] = table.NRFaddr[4];
				break;	//exit when time out,and do not change original address
			}

			SetRX_Mode();                 // reset RX mode write RX panel address
			delay_ms(4);									// delay is needed after reset NRF
		  sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
      
		  if((sta & 0x0E )== 0x00){
				NRFMatched = 1;
			}else{
				RX_ADDRESS[4] ++;		//search the next RX_ADDRESS
				if(RX_ADDRESS[4] == 0xff ){
					RX_ADDRESS[4] = 0x00;
				}
			}

  }while((sta & 0x0E )== 0x0E); 
	
	SetRX_Mode();                 // reset RX mode
	
	if((NRFMatched == 1)&&(RX_ADDRESS[4]!= table.NRFaddr[4])){
		SaveParamsToEEPROM();			//write eeprom when current addr != original addr
	}
	
	LED3_OFF;		// matching end 
}
*/

