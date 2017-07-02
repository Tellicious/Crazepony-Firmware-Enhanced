#include "I2C.h"

void I2C_write (uint8_t address, uint8_t thisRegister, uint8_t thisValue){
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as transmitter
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	// Send register to write to
	I2C_SendData(I2C1, thisRegister);
	//while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Send data to be written
	I2C_SendData(I2C1, thisValue);
	//while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Send STOP
	I2C_GenerateSTOP(I2C1, ENABLE);
}
	
void I2C_read (uint8_t address, uint8_t startRegister, uint8_t* buffer, uint8_t number_of_registers){
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as transmitter
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	// Send register to read from
	I2C_SendData(I2C1, startRegister);
	//while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)));
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as receiver
	I2C_Send7bitAddress(I2C1, address | (1 << 7), I2C_Direction_Receiver);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
	while (number_of_registers--){
		while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
		*buffer++ = I2C_ReceiveData(I2C1);
		if (number_of_registers == 1){
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}
	}
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}
