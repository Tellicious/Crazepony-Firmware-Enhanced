#ifndef _GLOBAL_VARIABLES_H_
#define _GLOBAL_VARIABLES_H_
#include "stdint.h"
#include "NRF24.h"

#pragma anon_unions //enables anon structures

//Used as indexes
#define THROTTLE 3
#define ROLL 0
#define PITCH 1
#define YAW 2

//Structure containing warnings
typedef union{
	uint16_t ALL;
	struct {
		uint8_t NOT_ARMED : 1;
		uint8_t LOW_BATTERY : 1;
		uint8_t BATTERY_CHARGING : 1;
		uint8_t IMU_NOT_CALIBRATED : 1;
		uint8_t LOST_RC : 1;
		uint8_t IMU_ST_NOT_PASSED : 1;
		uint8_t IMU_NOT_RESPONDING : 1;
		uint8_t CRASH : 1;
		uint8_t TILT_EXCEEDED : 1;
	};
} warnings_t;
extern warnings_t warnings;
 
//Structure containing flight modes
typedef union{
	uint8_t ALL;
	struct{
		uint8_t FLIGHT_ENABLED: 1;
		uint8_t IN_FLIGHT : 1;
		uint8_t LANDING : 1;
		uint8_t TAKEOFF: 1;
		uint8_t HEADLESS : 1;
		uint8_t ALT_HOLD : 1;
	};
} flightModes_t;
extern flightModes_t flightModes;

//NRF24 rx address
extern uint8_t NRF24_rx_address[5];
 
//Structure containing accel, gyro, mag, pressure, etc.
typedef struct {
	float raw_accel[3]; // m/s^2
	float raw_gyro[3]; // rad/s
	//float raw_mag[3]; // T
	float cal_accel[3]; //Calibrated values, m/s^2
	float cal_gyro[3]; //Calibrated values, rad/s
	//float cal_mag[3]; //Calibrated values, T
	float pressure;  // Pa
	float temperature; // °C
} INSData_t;
extern INSData_t INSData;
 
// Structure containing Attitude and Heading data
typedef struct{
	float roll; //rad
	float pitch; //rad
	float yaw; //rad
} AHRSData_t;
extern AHRSData_t AHRSData;
 
typedef struct{
	float altitude; //m
	float rateOfClimb; //m/s
	float takeOffPressure; //Pa
	float inv_referencePressure; //Pa
} NAVData_t;
extern NAVData_t NAVData;

typedef struct{
	uint16_t ADCVal; // ADC value 0-4096
	float voltage; // V, battery voltage
} Battery_t;
extern Battery_t Battery;

extern  uint8_t accUpdated;				

//IMU
typedef struct float_xyz
{
    float X;
    float Y;
    float Z;
    
}S_FLOAT_XYZ;

    

extern S_FLOAT_XYZ DIF_ACC;		//差分加速度
extern S_FLOAT_XYZ EXP_ANGLE;		//期望角度
extern S_FLOAT_XYZ DIF_ANGLE;		//期望角度与实际角度差
          
                
#endif
                
        



