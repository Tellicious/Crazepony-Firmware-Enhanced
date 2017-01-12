#ifndef _Led_H_
#define _Led_H_
#include "stm32f10x.h"


#define LED1_ON    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define LED1_OFF   GPIO_ResetBits(GPIOA, GPIO_Pin_11)


#define LED2_ON    GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LED2_OFF   GPIO_ResetBits(GPIOA, GPIO_Pin_8)


#define LED3_ON    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define LED3_OFF   GPIO_ResetBits(GPIOB, GPIO_Pin_1)


#define LED4_ON    GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define LED4_OFF   GPIO_ResetBits(GPIOB, GPIO_Pin_3)

void LEDFSM(void);
void PowerOn(void);

#endif

