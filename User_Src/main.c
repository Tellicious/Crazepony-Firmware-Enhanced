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
编写者：小马  (Camel) 、 祥(Samit)、Nieyong
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 5.10
初版时间: 2014-01-28
功能： 
1. 硬件驱动
2. 飞行控制：自稳、定高、智能头向、自动降落、故障保护
3. 支持App与2401 RC同时控制
4. App 与 PC端在线监控、无线调参
------------------------------------
*/
#include "SysConfig.h"
#include "config.h"        //包含所有的驱动头文件
#include "imu.h"
#include "Altitude.h"
#include "ConfigTable.h"
#include "IMUSO3.h"
#include "Control.h"
#include "FailSafe.h"
//#include "I2C.h"
#include "Init_Config.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
#include "stdint.h"
#include "NRF24.h"
#include "MS5611.h"
#include "Altitude_KF.h"
#include "SafetyChecks.h"
#include "Autopilot.h"

//sw counter
uint16_t  batCnt; 
//check executing time and period in different loop
uint32_t startTime[5],execTime[5];
uint32_t realExecPrd[5];	//us , real called period in different loop
uint8_t accUpdated=0;


/********************************************
              飞控主函数入口
功能：
1.初始化各个硬件
2.初始化系统参数
********************************************/
int main(void){
	HardwareInit();
#ifdef UART1_USE_INTERRUPTS
	UartInitVals();
#endif
	
	STMFLASH_Unlock();
	LoadParamsFromEEPROM();
	NRF24_config(40, NRF24_PA_MAX, NRF24_2MBPS, sizeof(RCRawData), NRF24_DIS_DYN_PYL, NRF24_ADDR_5, NRF24_CRC_16, NRF24_EN_AUT_ACK, NRF24_ARD_500, NRF24_ARC_15);
	NRF24_RXMatch(&NRF24_rx_address, 4, 10, 7500);
	MPU6050_initialize(MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_4, MPU6050_DLPF_BW_98);
	MS5611_Init(MS5611_OSR_4096, MS5611_OSR_4096);
	CtrlPIDInit();
	
	alt_KF_init(10); //100 Hz

	IMU_Init();			// sample rate and cutoff freq.  sample rate is too low now due to using dmp.
	MotorsPWMFlash(10,10,10,10);
	altCtrlMode=MANUAL;
	// Get current pressure and set it as reference pressure for ground level
	getTakeOffPressure();
	setReferencePressure(NAVData.takeOffPressure);
	
	
	while (1){
		//100Hz Loop
		if(loop100HzFlag){
				loop100HzFlag=0;
				realExecPrd[1]=micros()-startTime[1];
				startTime[1]=micros();
				
				IMUSO3Thread();
				accUpdated=1;
			
				MS5611_Thread();

				if(imuCaliFlag){
						if(IMU_Calibrate()){
							imuCaliFlag=0;
							gParamsSaveEEPROMRequset=1;	//请求记录到EEPROM
							imu.caliPass=1;
						}
				} 
				
				CtrlAttiRate();
				CtrlMotor();

				execTime[1]=micros()-startTime[1];
		}
		
		//Need to receive NRF2401 RC instantly
		
		if (NRF24_available()){
			NRF24_read(&RCRawData, sizeof(RCRawData));
			ReceiveDataFromRC();
			RCCheckRst();	
		}
		//50Hz Loop
		if(loop50HzFlag)
		{
				loop50HzFlag=0;
				realExecPrd[3]=micros()-startTime[3];
				startTime[3]=micros();
				
				RCDataProcess();
			  
				FlightModeFSMSimple();
				
				if(altCtrlMode==LANDING)	 
				{	  
						AutoLand();
				}
				
				//高度融合
		 		AltitudeCombineThread();

				CtrlAlti();		 

				CtrlAttiAng();	 
				
				execTime[3]=micros()-startTime[3];
		}
		
		//10Hz loop
		if(loop10HzFlag){
				loop10HzFlag=0; 
				realExecPrd[2]=micros()-startTime[2];
				startTime[2]=micros(); 
				//Check battery every BATT_CHK_PRD ds
				if((++batCnt) >= BATT_CHK_PRD){
					batCnt=0; 
					BatteryCheck();
				}
				//Check if RC is still connected
				RCCheck()
				//Check if tilt limit is exceeded
				TiltCheck();
				//Check if altitude limit is exceeded
				AltitudeCheck();
				//Command LEDs
				LEDFSM();
				
				//EEPROM Conifg Table request to write. 
				if(gParamsSaveEEPROMRequset){
						gParamsSaveEEPROMRequset=0;
						SaveParamsToEEPROM();
				}
				//失控保护，例如侧翻，丢失遥控信号等
				FailSafe();	
				execTime[2]=micros()-startTime[2];
		}
  }
}

