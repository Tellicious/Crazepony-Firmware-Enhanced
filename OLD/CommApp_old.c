#include "config.h"
#include "SysConfig.h"
#include "CommApp.h"
#include "imu.h"
#include "Altitude.h"
 
 
uint32_t lastGetRCTime;
uint8_t armState=DISARMED;//,disarmRequest=0;

 
#define MAX_LEN 32
volatile uint8_t UdataBuf[MAX_LEN];
const uint8_t HEADER[2]={0xAA,0x55};
uint16_t rcData[4]={1500,1500,1500,1500};

#define CONSTRAIN(x,min,max) {if(x<min) x=min; if(x>max) x=max;}
extern float dbScaleLinear(float x, float x_end, float deadband);

void RCDataProcess(void)
{
	//飞机电池过放，处于自动降落状态
	//对遥控数据不再响应，使用归中值
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
 
	RC_DATA.THROTTLE=rcData[THROTTLE]-1000;
	RC_DATA.YAW= YAW_RATE_MAX * dbScaleLinear((rcData[YAW] - 1500),500,APP_YAW_DB);
	RC_DATA.PITCH= Angle_Max * dbScaleLinear((rcData[PITCH] - 1500),500,APP_PR_DB);
	RC_DATA.ROOL= Angle_Max * dbScaleLinear((rcData[ROLL] - 1500),500,APP_PR_DB);
	
	switch(armState)
	{
		case REQ_ARM:
			if(IMUCheck() && !Battery.alarm){	
				armState=ARMED;
				FLY_ENABLE=0xA5;
			}else{
				FLY_ENABLE=0;
				armState=DISARMED;
			}
		break;
		case REQ_DISARM:
			FLY_ENABLE=0;
			altCtrlMode=MANUAL;		//上锁后加的处理
			zIntReset=1;
			thrustZSp=0;	
			thrustZInt=estimateHoverThru();
			offLandFlag=0;
			
			armState=DISARMED;
		break;
		default:
			break;
			
	}
	

}
