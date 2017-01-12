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
mpu6050.c file
------------------------------------
*/
#include "MPU6050.h"
#include "I2C.h"
#include "GlobalVariables.h"
#include "ConfigParams.h"
#include "config.h"
#include "string.h"
#include "delay.h"

#include "stdio.h"

float _sc_gyro, _sc_accel;

uint8_t MPU6050_initialize(uint8_t gyro_range, uint8_t accel_range, uint8_t LPF_BW) {
	uint8_t buf;
	I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_WHO_AM_I, 1, &buf); // Trash the first reading
	I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_WHO_AM_I, 1, &buf);
	if (buf != 0x68){
		return 0;
	}
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, 1 << 7); //Reset the MPU
	delay_ms(50); //Necessary when resetting the MPU
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_SMPLRT_DIV, 0x00);      //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) => 8kHZ if LPF disabled, otherwise 1kHz
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_INT_PIN_CFG, 1 << 6 | 1 << 5 | 1 << 4 | 1 << 1);  // INT_PIN_CFG   -- INT ACTIVE HIGH, OPEN-DRAIN, LATCH ENABLED, INT_RD_CLEAR ENABLED, I2C BYPASS ENABLED
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_INT_ENABLE, 0x01); //enable interrupt on DRDY
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_CONFIG, LPF_BW);  //selected Low-Pass Filter BandWidth
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_CONFIG, gyro_range); //Self-test off and selected gyro range, 2000 default
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_CONFIG, accel_range);	//8 default
	
	if (gyro_range == MPU6050_GYRO_FS_250){
		_sc_gyro = 133.1581e-6f;
	}
	else if (gyro_range == MPU6050_GYRO_FS_500){
		_sc_gyro = 266.3161e-6f;
	}
	else if (gyro_range == MPU6050_GYRO_FS_1000){
		_sc_gyro = 532.6322e-6f;
	}
	else if (gyro_range == MPU6050_GYRO_FS_2000){
		_sc_gyro = 1.065264e-3f;
	}
	else{
		_sc_gyro = 0;
	}
	
	if (accel_range ==  MPU6050_ACCEL_FS_2){
		_sc_accel = INS_G_VAL / 16384.0f;
	}
	else if (accel_range ==  MPU6050_ACCEL_FS_4){
		_sc_accel = INS_G_VAL / 8192.0f;
	}
	else if (accel_range ==  MPU6050_ACCEL_FS_8){
		_sc_accel = INS_G_VAL / 4096.0f;
	}
	else if (accel_range ==  MPU6050_ACCEL_FS_16){
		_sc_accel = INS_G_VAL / 2048.0f;
	}
	return 1;
}

void MPU6050_read_raw_accel(void) {
    uint8_t buf[6];
    I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
    INSData.raw_accel[0] = (float) ((int16_t)((buf[0] << 8) | buf[1])) * _sc_accel;
    INSData.raw_accel[1] = (float) ((int16_t)((buf[2] << 8) | buf[3])) * _sc_accel;
    INSData.raw_accel[2] = (float) ((int16_t)((buf[4] << 8) | buf[5])) * _sc_accel;
}

void MPU6050_read_raw_gyro(void) {
    uint8_t buf[6];
    I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_XOUT_H, 6, buf);
    INSData.raw_gyro[0] = (float) ((int16_t)((buf[0] << 8) | buf[1])) * _sc_gyro;
    INSData.raw_gyro[1] = (float) ((int16_t)((buf[2] << 8) | buf[3])) * _sc_gyro;
    INSData.raw_gyro[2] = (float) ((int16_t)((buf[4] << 8) | buf[5])) * _sc_gyro;
}

void MPU6050_read_raw_data(void) {
	uint8_t buf[14];
    I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, buf);
    INSData.raw_accel[0] = (float) ((int16_t)((buf[0] << 8) | buf[1])) * _sc_accel;
    INSData.raw_accel[1] = (float) ((int16_t)((buf[2] << 8) | buf[3])) * _sc_accel;
    INSData.raw_accel[2] = (float) ((int16_t)((buf[4] << 8) | buf[5])) * _sc_accel;
		INSData.raw_gyro[0] = (float) ((int16_t)((buf[8] << 8) | buf[9])) * _sc_gyro;
    INSData.raw_gyro[1] = (float) ((int16_t)((buf[10] << 8) | buf[11])) * _sc_gyro;
    INSData.raw_gyro[2] = (float) ((int16_t)((buf[12] << 8) | buf[13])) * _sc_gyro;
	printf("Acc x: %3.3f, y: %3.3f, z: %3.3f, Gyro x: %3.3f, y: %3.3f, z: %3.3f\r\n",INSData.raw_accel[0], INSData.raw_accel[1], INSData.raw_accel[2], INSData.raw_gyro[0], INSData.raw_gyro[1], INSData.raw_gyro[2]);
}

uint8_t MPU6050_status(void) {
	uint8_t buf;
    I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_INT_STATUS, 1, &buf);
    return buf;
}

uint8_t MPU6050_read_DRDY(uint32_t timeout){
	uint32_t now = micros();
    while((micros() - now) < timeout){
      if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET){
        MPU6050_read_raw_data();
        return 1;
      }
      if ((int32_t) (micros() - now) < 0){
        now = 0L;
      }
    }
    return 0;
}

uint8_t MPU6050_read_STATUS(uint32_t timeout){
    uint32_t now = micros();
    while ((micros() - now) < timeout){
      if (MPU6050_status() & 0x01){
        MPU6050_read_raw_data();
        return 1;
      }
      if ((int32_t) (micros() - now) < 0){
        now = 0L;
      }
    }
    return 0;
}

uint8_t MPU6050_self_test(void){
	uint16_t repeats = 100;
	// factory trim values
	uint8_t trims[4], atrim[3], gtrim[3], i;
	float accel_ftrim[3];
	float gyro_ftrim[3];
	// previous configurations
	uint8_t gyro_conf, accel_conf;
	float _sc_g_old, _sc_a_old;
	// Errors
	float err_g, err_a;
	// pre- and post- ST values
	float accel_prev[3], gyro_prev[3];
	float accel_post[3], gyro_post[3];
	memset(accel_prev, 0, sizeof(accel_prev));
	memset(gyro_prev, 0, sizeof(gyro_prev));
	memset(accel_post, 0, sizeof(accel_post));
	memset(gyro_post, 0, sizeof(gyro_post));
	// saving previous configurations
	I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_CONFIG, 1, &gyro_conf);
	_sc_g_old = _sc_gyro;
	I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_CONFIG, 1, &accel_conf);
	_sc_a_old = _sc_accel;
	// gyro self test has to be done at 250DPS
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250);
	_sc_gyro = 1;
	// accel self test has to be done at 8g
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_8);
	_sc_accel = 1;
	//get pre-ST values
	for(i = 0; i < repeats; i++){
		delay_ms(1);
		MPU6050_read_raw_data();
		accel_prev[0] += INSData.raw_accel[0];
		accel_prev[1] += INSData.raw_accel[1];
		accel_prev[2] += INSData.raw_accel[2];
		gyro_prev[0] += INSData.raw_gyro[0];
		gyro_prev[1] += INSData.raw_gyro[1];
		gyro_prev[2] += INSData.raw_gyro[2];
	}
	// start Self-test
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250 | 0xE0);
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_8 | 0xE0);
	delay_ms(20);
	// get post-ST values
	for(i = 0; i < repeats; i++){
		delay_ms(1);
		MPU6050_read_raw_data();
		accel_post[0] += INSData.raw_accel[0];
		accel_post[1] += INSData.raw_accel[1];
		accel_post[2] += INSData.raw_accel[2];
		gyro_post[0] += INSData.raw_gyro[0];
		gyro_post[1] += INSData.raw_gyro[1];
		gyro_post[2] += INSData.raw_gyro[2];
	}
	//calculate mean value
	for (i = 0; i < 3; i++) {
		accel_prev[i] /= repeats;
		gyro_prev[i] /= repeats;
		accel_post[i] /= repeats;
		gyro_post[i] /= repeats;
	}
	// extract factory trim values
	I2C_read(MPU6050_I2C_ADDR, MPU6050_RA_SELF_TEST_X, 4, trims);
	atrim[0] = ((trims[0] >> 3) & 0x1C) | ((trims[3] >> 4) & 0x03);
	atrim[1] = ((trims[1] >> 3) & 0x1C) | ((trims[3] >> 2) & 0x03);
	atrim[2] = ((trims[2] >> 3) & 0x1C) | ((trims[3] >> 0) & 0x03);
	gtrim[0] = trims[0] & 0x1F;
	gtrim[1] = trims[1] & 0x1F;
	gtrim[2] = trims[2] & 0x1F;
	// convert factory trims to right units
	for (i = 0; i < 3; i++) {
		accel_ftrim[i] = 4096 * 0.34f * powf(0.92f / 0.34f, (atrim[i] - 1) / 30.0f);
		gyro_ftrim[i] = 25 * 131.0f * powf(1.046f, gtrim[i] - 1);
	}
	// Y gyro trim is negative
	gyro_ftrim[1] *= -1;
	// restart standard operation
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_CONFIG, gyro_conf);
	_sc_gyro = _sc_g_old;
	I2C_write(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_CONFIG, accel_conf);
	_sc_accel = _sc_a_old;
	//check errors
	for (i = 0; i < 3; i++) {
		err_g = 100 * (gyro_post[i] - gyro_prev [i] - gyro_ftrim[i]) / gyro_ftrim [i];
		err_a = 100 * (accel_post[i] - accel_prev [i] - accel_ftrim[i]) / accel_ftrim [i];
		if ((fabsf(err_a) > 14) | (fabsf(err_g) > 14)){
			return 0;
		}
	}
	return 1;
}
