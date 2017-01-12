#ifndef _I2C_H_
#define _I2C_H_
#include "stm32f10x.h"

/*
	SCL-->PB6
	SDA-->PB7
*/

void I2C_write (uint8_t address, uint8_t thisRegister, uint8_t thisValue);
void I2C_sendCMD(uint8_t address, uint8_t thisCMD);
void I2C_read (uint8_t address, uint8_t startRegister, uint8_t number_of_registers, uint8_t* buffer);
void I2C_writeBytes (uint8_t address, uint8_t startRegister, uint8_t number_of_registers, uint8_t* buffer);
void I2C_writeBits (uint8_t address, uint8_t thisRegister, uint8_t startBit, uint8_t number_of_bits, uint8_t data);

#endif
