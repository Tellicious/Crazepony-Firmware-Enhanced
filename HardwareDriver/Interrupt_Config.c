#include "ConfigParams.h"
#include "stm32f10x.h"
#include "stdio.h"
#include "UART1.h"
#include "config.h"
#include "imu.h"
#include "control.h"
#include "GlobalVariables.h"
#include "LED.h"

uint8_t  loop200HzFlag, loop100HzFlag, loop50HzFlag, loop20HzFlag, loop10HzFlag;
volatile uint16_t loop200HzCnt, loop100HzCnt, loop50HzCnt,loop20HzCnt, loop10HzCnt;

void TIM4_IRQHandler(void){
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ){     
					if (++loop200HzCnt >= 5){
							loop200HzCnt=0;
							loop200HzFlag=1;
					}
					if (++loop100HzCnt >= 10){
							loop100HzCnt=0;
							loop100HzFlag=1;
					}
					if(++loop50HzCnt >= 20){
							loop50HzCnt=0;
							loop50HzFlag=1;
					}
					if(++loop20HzCnt >= 50 ){
							loop20HzCnt=0;
							loop20HzFlag=1;
					}
					if(++loop10HzCnt >= 100 ){
							loop10HzCnt=0;
							loop10HzFlag=1;
					}
          
          TIM_ClearITPendingBit(TIM4 , TIM_IT_Update);   
    }
}

#ifdef UART_DEBUG
float hoverThru;
extern uint8_t NRF24_rx_address[5];
void TIM3_IRQHandler(void) {
    if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) {     
				// DO NOT USE THE FOLLOWING LINE, OTHERWISE IT GETS STUCK
				//GetBattVolt();
				printf("\r\n");
				/*printf("RC Throttle: %d\r\n", rcData[THROTTLE]);
				printf("RC Yaw: %d\r\n", rcData[YAW]);
				printf("RC Pitch: %d\r\n", rcData[PITCH]);
				printf("RC Roll: %d\r\n", rcData[ROLL]);*/
				printf(" Yaw ---> %5.2f degree\r\n",(float)imu.yaw);
				printf(" Pitch---> %5.2f degree\r\n",(float)imu.pitch);
				printf(" Roll ---> %5.2f degree\r\n",(float)imu.roll);
				printf("====================================\r\n");
				/*printf(" Motor M1 PWM---> %d\r\n",TIM2->CCR1);
				printf(" Motor M2 PWM---> %d\r\n",TIM2->CCR2);
				printf(" Motor M3 PWM---> %d\r\n",TIM2->CCR3);
				printf(" Motor M4 PWM---> %d\r\n",TIM2->CCR4);
				printf("====================================\r\n");*/
				printf(" Pressure ---> %5.2f Pa\r\n", INSData.pressure);
				//printf(" Altitude ---> %5.2f m\r\n", MS5611_Altitude);
				printf(" Temperature---> %5.2f °C\r\n", INSData.temperature);
				/*printf("====================================\r\n");
				printf(" Battery Voltage---> %3.2fV\r\n",Battery.voltage);
				
				hoverThru = estimateHoverThru();
				printf(" Hover Thru---> %3.2f\r\n",hoverThru);
				printf(" RX Addr ---> 0x%x\r\n",NRF24_rx_address[4]);*/
				printf("====================================\r\n");
        TIM_ClearITPendingBit(TIM3 , TIM_IT_Update);  
    }
}
#endif

// UART1 Interrupt
#ifdef UART1_USE_INTERRUPTS
volatile uint8_t Udatatmp;
void USART1_IRQHandler(void) {
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET){   
		USART_SendData(USART1, UartBuf_RD(&UartTxbuf));
		if(UartBuf_Cnt(&UartTxbuf) == 0)  
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
	else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		Udatatmp = (uint8_t) USART_ReceiveData(USART1);
		UartBuf_WD(&UartRxbuf,Udatatmp);
	}
}
#endif
