#ifndef _SPI_H_
#define _SPI_H_
#include "stdint.h"
/*
NRF24:       
		CE:PA15
		CSN:PA4
		SCK:PA5
		MOSI:PA7
		MISO:PA6
*/
#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_15) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_15)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)

uint8_t SPI_RW(uint8_t dat);

#endif

