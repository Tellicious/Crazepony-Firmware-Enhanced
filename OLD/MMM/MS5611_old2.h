#ifndef __MS5611_H
#define __MS5611_H
#include "stdint.h"

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00  //Conversion time 0.6ms  Resolution 0.065mbar
#define MS5611_OSR_512 0x02  //Conversion time 1.2ms  Resolution 0.042mbar
#define MS5611_OSR_1024 0x04 //Conversion time 2.3ms  Resolution 0.027mbar
#define MS5611_OSR_2048 0x06 //Conversion time 4.6ms  Resolution 0.018mbar
#define MS5611_OSR_4096 0x08 //Conversion time 9.1ms  Resolution 0.012mbar


//  Temperature in 1C
//  Pressure    in 0.01mbar = Pa
//  Altitude    in meter
//  VerticalSpeed in m/s
extern volatile float MS5611_Altitude;
extern uint8_t Baro_ALT_Updated;
extern uint8_t paOffsetInited;

void MS5611_Init(uint8_t press_OSR, uint8_t temp_OSR);
void MS5611_ThreadNew(void) ;
uint8_t  WaitBaroInitOffset(void);

#endif

//------------------End of File----------------------------
