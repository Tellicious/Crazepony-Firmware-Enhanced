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
#include "control.h"
#include "FailSafe.h"
#include "CommApp.h"
#include "I2C.h"
#include "Init_Config.h"
#include "Config_Params.h"
#include "stdint.h"
#include "NRF24.h"

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
int main(void)
{

	HardwareInit();
#ifdef UART1_USE_INTERRUPTS
	UartInitVals();
#endif
	PowerOn();
	STMFLASH_Unlock();            //内部flash解锁
  LoadParamsFromEEPROM();
	NRF24_config(40, NRF24_PA_MAX, NRF24_2MBPS, 32, NRF24_DIS_DYN_PYL, NRF24_ADDR_5, NRF24_CRC_16, NRF24_EN_AUT_ACK, NRF24_ARD_500, NRF24_ARC_15);
	NRF24_RXMatch(&NRF24_rx_address, 4, 10, 7500);
	
  //MotorInit();	                //马达初始化
  //BatteryCheckInit();           //电池电压监测初始化
  IIC_Init();                   //IIC初始化
	MPU6050_initialize();

  //NRF24L01_INIT();              //NRF24L01初始化
 
	BatteryCheck();

	MS5611_Init();

	IMU_Init();			// sample rate and cutoff freq.  sample rate is too low now due to using dmp.
	
	MotorsPWMFlash(10,10,10,10);
		
	altCtrlMode=MANUAL;
	WaitBaroInitOffset();		//等待气压初始化高度完成
	
  while (1){
		//100Hz Loop
		//Crazepony默认使用100Hz的控制频率
		if(loop100HzFlag){
				loop100HzFlag=0;
				
				realExecPrd[1]=micros()-startTime[1];
				startTime[1]=micros();
				
				IMUSO3Thread();
				accUpdated=1;
			
				//气压读取
				MS5611_ThreadNew();		//FSM, take aboue 0.5ms some time

				//imu校准
				if(imuCaliFlag)
				{
						if(IMU_Calibrate())
						{
							imuCaliFlag=0;
							gParamsSaveEEPROMRequset=1;	//请求记录到EEPROM
							imu.caliPass=1;
						}
				} 
				
				CtrlAttiRate();
				CtrlMotor();

				execTime[1]=micros()-startTime[1];
		}
		
		//Need to recieve 2401 RC instantly so as to clear reg.
		//Nrf_Irq();
		
		if (NRF24_available()){
			NRF24_read(&NRF24L01_RXDATA, sizeof(NRF24L01_RXDATA));
			ReceiveDataFormNRF();
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
		if(loop10HzFlag)
		{
				loop10HzFlag=0; 
				realExecPrd[2]=micros()-startTime[2];
				startTime[2]=micros(); 
			
				//Check battery every BAT_CHK_PRD ms
				if((++batCnt) * 100 >=BAT_CHK_PRD) 
				{
					batCnt=0; 
					BatteryCheck();
				}
				
				
				//EEPROM Conifg Table request to write. 
				if(gParamsSaveEEPROMRequset)
				{
						gParamsSaveEEPROMRequset=0;
						SaveParamsToEEPROM();
				}

				//失控保护，例如侧翻，丢失遥控信号等
				FailSafe();	
				
				LEDFSM();
				
				execTime[2]=micros()-startTime[2];
		}
  }
}

