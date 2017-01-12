#include "I2C.h"

void I2C_write (uint8_t address, uint8_t thisRegister, uint8_t thisValue){
	// Wait for the bus to be free
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
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
	//while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
	// Send data to be written
	I2C_SendData(I2C1, thisValue);
	//while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Send STOP
	I2C_GenerateSTOP(I2C1, ENABLE);
	// Wait for I2C to stop
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

void I2C_sendCMD(uint8_t address, uint8_t thisCMD){
		// Wait for the bus to be free
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as transmitter
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	// Send register to write to
	I2C_SendData(I2C1, thisCMD);
	//while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Send STOP
	I2C_GenerateSTOP(I2C1, ENABLE);
	// Wait for I2C to stop
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

/*
void I2C_read (uint8_t address, uint8_t startRegister, uint8_t number_of_registers, uint8_t* buffer){
	// Wait for the bus to be free
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as transmitter
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	// Send register to read from
	I2C_SendData(I2C1, startRegister);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Enable Acknowledgement
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as receiver
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Receiver);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
	while (number_of_registers--){
		while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
		*buffer++ = I2C_ReceiveData(I2C1);
		if (number_of_registers == 1){
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}
	}
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}*/

void I2C_read (uint8_t address, uint8_t startRegister, uint8_t number_of_registers, uint8_t* buffer){
	// Wait for the bus to be free
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as transmitter
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	// Send register to read from
	I2C_SendData(I2C1, startRegister);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
	// Enable Acknowledgement, clear POS flag
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	// Initiate Start Sequence (wait for EV5)
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	// Send Address
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Receiver);
	// EV6
	while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR));
	if (number_of_registers == 1){
		// Clear Ack bit
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		// EV6_1 -- must be atomic -- Clear ADDR, generate STOP
		__disable_irq();
		(void) I2C1->SR2;
		I2C_GenerateSTOP(I2C1, ENABLE);
		__enable_irq();
		// Receive data EV7
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE));
		*buffer++ = I2C_ReceiveData(I2C1);
	}
	else if (number_of_registers == 2) {
		// Set POS flag
		I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Next);
		// EV6_1 -- must be atomic and in this order
		__disable_irq();
		(void) I2C1->SR2; // Clear ADDR flag
		I2C_AcknowledgeConfig(I2C1, DISABLE); // Clear Ack bit
		__enable_irq();
		// EV7_3 -- Wait for BTF, program stop, read data twice
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
		__disable_irq();
		I2C_GenerateSTOP(I2C1,ENABLE);
		*buffer++ = I2C1->DR;
		__enable_irq();
		*buffer++ = I2C1->DR;
	}
	else {
		(void) I2C1->SR2; // Clear ADDR flag
		while (number_of_registers-- != 3) {
			// EV7 -- cannot guarantee 1 transfer completion time, wait for BTF
			// instead of RXNE
			while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
			*buffer++ = I2C_ReceiveData(I2C1);
		}
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
		// EV7_2 -- Figure 1 has an error, doesn't read N-2 !
		I2C_AcknowledgeConfig(I2C1, DISABLE); // clear ack bit
		__disable_irq();
		*buffer++ = I2C_ReceiveData(I2C1); // receive byte N-2
		I2C_GenerateSTOP(I2C1, ENABLE); // program stop
		__enable_irq();
		*buffer++ = I2C_ReceiveData(I2C1); // receive byte N-1
		// wait for byte N
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		*buffer++ = I2C_ReceiveData(I2C1);
		number_of_registers = 0;
	}
// Wait for stop
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

void I2C_writeBytes (uint8_t address, uint8_t startRegister, uint8_t number_of_registers, uint8_t* buffer){
	// Wait for the bus to be free
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	// Send START
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)));
	// Send address as transmitter
	I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	// Send register to write to
	I2C_SendData(I2C1, startRegister);
	while (number_of_registers--) {
		// wait on BTF
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
		I2C_SendData(I2C1, *buffer++);
	}
	while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF));
	// Send STOP
	I2C_GenerateSTOP(I2C1, ENABLE);	
	// Wait for I2C to stop
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

void I2C_writeBits (uint8_t address, uint8_t thisRegister, uint8_t startBit, uint8_t number_of_bits, uint8_t data){
    uint8_t b, mask;
    I2C_read(address, thisRegister, 1, &b);
	mask = (0xFF << (startBit + 1)) | 0xFF >> (7 - startBit + number_of_bits);
	data <<= (startBit - number_of_bits + 1);
	b &= mask;
	b |= data;
	I2C_write(address, thisRegister, b);
}
