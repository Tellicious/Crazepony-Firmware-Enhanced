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
Filename:	imu.c
Author:		祥 、小马、nieyong
------------------------------------
*/
#include "config.h"
#include "imu.h"
#include "filter.h"
#include "SysConfig.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"


imu_t imu={0};
uint8_t imuCaliFlag=0;

void IMU_Init(void)
{
		imu.ready=0;
		imu.caliPass=1;
		//filter rate
		LPF2pSetCutoffFreq_1(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);		//30Hz
		LPF2pSetCutoffFreq_2(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
		LPF2pSetCutoffFreq_3(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
		LPF2pSetCutoffFreq_4(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
		LPF2pSetCutoffFreq_5(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
		LPF2pSetCutoffFreq_6(IMU_SAMPLE_RATE,IMU_FILTER_CUTOFF_FREQ);
}


//should place to a level surface and keep it stop for 1~2 second
//return 1 when finish
uint8_t IMU_Calibrate(void)
{
	//3s 
	static float accSum[3]={0,0,0};
	static float gyroSum[3]={0,0,0};
	static uint16_t cnt=0;
	static uint16_t tPrev=0;
	static uint8_t calibrating=0;
	uint8_t ret=0;
	uint8_t i=0;
	uint16_t dt=0,now=0;

	now=millis();
	dt=now-tPrev;	

	if(calibrating==0)
	{
			calibrating=1;
			for(i=0;i<3;i++)
			{
					accSum[i]=0;
					gyroSum[i]=0;
					cnt=0;
					imu.ready=0;
			}
			
	}
	
	if(dt>=10)		//10ms 
	{
			if(cnt<300)
			{
				for(i=0;i<3;i++)
				{
					accSum[i]+=INSData.raw_accel[i];		
					gyroSum[i]+=INSData.raw_gyro[i];
				}
				cnt++;
				tPrev=now;
			}
			else
			{
					for(i=0;i<3;i++)
					{
						imu.accOffset[i]=accSum[i]/(float)cnt;
						imu.gyroOffset[i]=gyroSum[i]/(float)cnt;
					} 
					
					imu.accOffset[2]=imu.accOffset[2] - INS_G_VAL;
					
					calibrating=0;
					ret=1;
					//tobe added: write to eeprom !!
			}
	}

	return ret;
}

void ReadIMUSensorHandle(void){
		//read raw
		MPU6050_read_STATUS(MPU_READ_TIMEOUT);		
		imu.accb[0]=LPF2pApply_1(INSData.raw_accel[0]-imu.accOffset[0]);
		imu.accb[1]=LPF2pApply_2(INSData.raw_accel[1]-imu.accOffset[1]);
		imu.accb[2]=LPF2pApply_3(INSData.raw_accel[2]-imu.accOffset[2]);
		imu.gyro[0]=LPF2pApply_4(INSData.raw_gyro[0]);
		imu.gyro[1]=LPF2pApply_5(INSData.raw_gyro[1]);
		imu.gyro[2]=LPF2pApply_6(INSData.raw_gyro[2]); 

} 

#define ACCZ_ERR_MAX  0.05		//m/s^2
#define CHECK_TIME 5
uint8_t IMUCheck(void)
{
	  float accZSum=0;
	  uint8_t i;
	  float accZb=0;
	  
	for(i=0;i<CHECK_TIME;i++)
  {
		MPU6050_read_STATUS(MPU_READ_TIMEOUT);
		accZSum += INSData.raw_accel[2];
	} 

	accZb = accZSum / CHECK_TIME - imu.accOffset[2];	
	
	if(fabsf(accZb - INS_G_VAL) < ACCZ_ERR_MAX)
		imu.caliPass=1;
	else
		imu.caliPass=0;

	return imu.caliPass;
		
}
