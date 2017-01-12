#include "MS5611.h"
#include <math.h>
#include "stm32f10x_it.h"
#include "I2C.h"
#include "delay.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"

#define MS5611_ADDR         0xEE    // default I2C address
// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E
#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2








/**************************实现函数********************************************
*函数原型:		float MS5611_getAltitude(void)
*功　　能:	    将当前的气压值转成 高度。	 
*******************************************************************************/
float MS5611_getAltitude(void)
{
	float Altitude;
	static uint8_t paInitCnt = 0;
	static float Alt_Offset_Pa = 0; 
	static float paOffsetNum = 0; 
	// 是否初始化过0米气压值？
	if(Alt_Offset_Pa == 0)
	{ 
		if(paInitCnt > 50)
		{
			Alt_Offset_Pa = paOffsetNum / paInitCnt;
			paOffsetInited=1;
		}
		else
			paOffsetNum += INSData.pressure;
		
		paInitCnt++;
		
		Altitude = 0; //高度 为 0
		
		return Altitude;
	}
	//计算相对于上电时的位置的高度值 。单位为m
	Altitude = 4433000.0 * (1 - powf((INSData.pressure / Alt_Offset_Pa), 0.1903))*0.01f;
	
	return Altitude; 
}

// defines
#define READ_TEMPERATURE 0x25
#define READ_PRESSURE 0x52

// private variables
static uint8_t _TEMP_CMD, _PRESS_CMD;
static uint32_t _temp_delay, _press_delay;
static uint16_t _PROM_C[6];

void MS5611_readPROM(void) {
	uint8_t buf[2];
	int i;
	for (i = 0; i < 6; i++) {
		I2C_read (MS5611_ADDR, MS5611_PROM_BASE_ADDR + (i * 2), 2, buf);
		_PROM_C[i] = ((uint16_t) buf[0] << 8) | buf[1];
	}
}

void MS5611_Reset(void) {
	I2C_sendCMD(MS5611_ADDR, MS5611_RESET);
}

uint32_t MS5611_getConversion(void) {
	uint8_t buf[3];
	I2C_read(MS5611_ADDR, 0x00, 3, buf);
	return (((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + (uint32_t)buf[2]);
}

void MS5611_Init(uint8_t press_OSR, uint8_t temp_OSR) {  
	MS5611_Reset(); 
	delay_ms(100);
	MS5611_readPROM();
	_TEMP_CMD = MS5611_D2 + temp_OSR;
	switch (temp_OSR){
		case MS5611_OSR_256:
		_temp_delay = 1000;
		break;
		case MS5611_OSR_512:
		_temp_delay = 1500;
		break;
		case MS5611_OSR_1024:
		_temp_delay = 2500;
		break;
		case MS5611_OSR_2048:
		_temp_delay = 5000;
		break;
		case MS5611_OSR_4096:
		_temp_delay = 9500;
		break;
		default:
		_temp_delay = 9500;
		break;
	}
	_PRESS_CMD = MS5611_D1 + press_OSR;
	switch (press_OSR){
		case MS5611_OSR_256:
		_press_delay = 1000;
		break;
		case MS5611_OSR_512:
		_press_delay = 1500;
		break;
		case MS5611_OSR_1024:
		_press_delay = 2500;
		break;
		case MS5611_OSR_2048:
		_press_delay = 5000;
		break;
		case MS5611_OSR_4096:
		_press_delay = 9500;
		break;
		default:
		_press_delay = 9500;
		break;
	}
	NAVData.referencePressure = INS_QNE_VAL;
}

float calculateAltitude(void){
	return (44330.0f * (1 - powf((INSData.pressure / NAVData.referencePressure), 0.1903)));
}

void MS5611_tempCompPressure(int32_t ADCPress, int32_t ADCTemp) {
	// second-order temperature compensation, according to the datasheet
	int32_t TEMP, T2;
	int64_t OFF, SENS, Aux, OFF2, SENS2;
	int32_t dT = (int32_t)ADCTemp - ((int32_t)_PROM_C[4] << 8);
	
	TEMP = 2000 + (int32_t)(((int64_t)dT * _PROM_C[5]) >> 23);
	OFF  = ((int64_t)_PROM_C[1] << 16) + (((int64_t)_PROM_C[3] * dT) >> 7);
	SENS = ((int64_t)_PROM_C[0] << 15) + (((int64_t)_PROM_C[2] * dT) >> 8);
	
	if (TEMP < 2000){
		T2 = (dT * dT) >> 31;
		Aux = ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000);
		OFF2 = (5 * Aux) >> 1;
		SENS2 = (5 * Aux) >> 2;
		if (TEMP < -1500){
			Aux = ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
			OFF2 += (7 * Aux);
			SENS2 += (11 * Aux) >> 1;
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}

	INSData.pressure = (((ADCPress * SENS) >> 21) - OFF) >> 15;
	INSData.temperature = TEMP * 0.01f;
	MS5611_Altitude = MS5611_getAltitude(); 
}

uint8_t MS5611_Thread(void) {
	static uint8_t _taskToDo = 0;
	static uint32_t ADCTemp, ADCPress, _EndConversionTime;
	if (_taskToDo == READ_TEMPERATURE) {
		if(micros() > _EndConversionTime) {
			ADCTemp = MS5611_getConversion();	
			I2C_sendCMD(MS5611_ADDR, _PRESS_CMD);
			_EndConversionTime = micros() + _press_delay;
			_taskToDo = READ_PRESSURE;
		}
	}
 	else if (_taskToDo == READ_PRESSURE) {
		if(micros() > _EndConversionTime) {
			ADCPress = MS5611_getConversion();
			MS5611_tempCompPressure(ADCPress, ADCTemp);
			Baro_ALT_Updated = 0xff;
			I2C_sendCMD(MS5611_ADDR, _TEMP_CMD);
			_EndConversionTime = micros() + _temp_delay;
			_taskToDo = READ_TEMPERATURE;
			return 1;
		}
	}
	else {
		I2C_sendCMD(MS5611_ADDR, _TEMP_CMD);
		_EndConversionTime = micros() + _temp_delay;
		_taskToDo = READ_TEMPERATURE;
	}
	return 0;
}

void getTakeOffPressure(void){
	uint8_t samples = 0, toBeDiscarded = 10;
	NAVData.takeOffPressure = 0;
	// discard at least toBeDiscarded pressure samples
	while (toBeDiscarded){
		if(MS5611_Thread()){
			toBeDiscarded--;
		}
	}
	while (samples < 50){
		if(MS5611_Thread()){
			NAVData.takeOffPressure += INSData.pressure;
			samples++;
		}
	}
	NAVData.takeOffPressure /= samples;
}

void setReferencePressure(float referencePressureVal){
	NAVData.referencePressure = referencePressureVal;
	return;
}



















// 气压计状态机
#define SCTemperature  0x01	  //开始 温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03	  //开始转换 气压
#define SCPressureing  0x04	  //正在转换气压值
static uint8_t  Now_doing = SCTemperature;
uint8_t paOffsetInited=0;
uint8_t Baro_ALT_Updated = 0; //气压计高度更新完成标志。
volatile float MS5611_Altitude;

void MS5611_ThreadNew(void) {
	static uint32_t ADCTemp, ADCPress, _EndConversionTime;
	switch(Now_doing) {
 		case SCTemperature:
 			I2C_sendCMD(MS5611_ADDR, _TEMP_CMD);
			_EndConversionTime = micros() + _temp_delay;
			Now_doing = CTemperatureing;
			break;
		case CTemperatureing:
			if(micros() > _EndConversionTime) {
				ADCTemp = MS5611_getConversion();	
				I2C_sendCMD(MS5611_ADDR, _PRESS_CMD);
				_EndConversionTime = micros() + _press_delay;
				Now_doing = SCPressureing;
			}
			break;
 
		case SCPressureing:
			if(micros() > _EndConversionTime) {
				ADCPress = MS5611_getConversion();
				MS5611_tempCompPressure(ADCPress, ADCTemp);
				Baro_ALT_Updated = 0xff;
				I2C_sendCMD(MS5611_ADDR, _TEMP_CMD);
				_EndConversionTime = micros() + _temp_delay;
				Now_doing = CTemperatureing;
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}

//注意，使用前确保
uint8_t  WaitBaroInitOffset(void) {
	uint32_t t_end = micros() + 50 * 50000;
  while(!paOffsetInited){
			MS5611_ThreadNew();
			if(micros() >= t_end) {
				return 0;
			}
	}
	return 1;
}


//------------------End of File----------------------------
