#ifndef __UART1_H_
#define __UART1_H_

#include "stdint.h"
#include "ConfigParams.h"

#define RX_BUFFER_SIZE   512
#define TX_BUFFER_SIZE   512


#ifdef UART1_USE_INTERRUPTS
// USART Receiver buffer
typedef struct {
  uint16_t volatile Wd_Indx;
  uint16_t volatile Rd_Indx;
  uint16_t Mask;
  uint8_t *pbuf;
}UartBuf;

extern UartBuf UartTxbuf;
extern UartBuf UartRxbuf;
extern unsigned char rx_buffer[RX_BUFFER_SIZE];
extern unsigned char tx_buffer[TX_BUFFER_SIZE];

void UartInitVals(void);
void UartBufClear(UartBuf *Ringbuf);
extern void UartBuf_WD(UartBuf *Ringbuf,uint8_t DataIn);
extern uint8_t UartBuf_RD(UartBuf *Ringbuf);
extern uint16_t UartBuf_Cnt(UartBuf *Ringbuf);
void UartSendBuffer(uint8_t *dat, uint8_t len);
void UartPutCh(uint8_t ch);
#endif

#endif


