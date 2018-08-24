/*
      ____                       ____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
INS.c file
*/

#include "INS.h"
#include "stdint.h"
#include "ConfigParams.h"
#include "GlobalVariables.h"
#include "MPU6050.h"

float gyro_bias[3];
float gyro_bias_limit = 20 * TORAD; //according to MPU6050 datasheet ZRO should be less than 20 deg/s

uint8_t calibrateGyro(uint8_t number_of_measures){
	float bx_tmp = 0, by_tmp = 0, bz_tmp = 0;
	uint8_t to_be_discarded = 200;
	uint8_t ii;
	//discard some measures
	for (ii = 0; ii < to_be_discarded; ii++){
		MPU6050_read_STATUS(MPU_READ_TIMEOUT);
	}
	//perform acquisition and calibration
	for (ii = 0; ii < number_of_measures; ii++){
		if (MPU6050_read_STATUS(MPU_READ_TIMEOUT)){
			bx_tmp += INSData.raw_gyro[0];
			by_tmp += INSData.raw_gyro[1];
			bz_tmp += INSData.raw_gyro[2];
		}
		else{
			warnings.IMU_NOT_CALIBRATED = 1;
			return;
		}
	}
	gyro_bias[0] = bx_tmp / number_of_measures;
	gyro_bias[1] = by_tmp / number_of_measures;
	gyro_bias[2] = bz_tmp / number_of_measures;
	//check if biases are reasonable
	if ((absf(gyro_bias[0]) < gyro_bias_limit) && ((absf(gyro_bias[1]) < gyro_bias_limit)) && ((absf(gyro_bias[2]) < gyro_bias_limit))){
		warnings.IMU_NOT_CALIBRATED = 0;
	}
	else{
		warnings.IMU_NOT_CALIBRATED = 1;
	}
	return;
}
