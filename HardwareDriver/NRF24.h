//  NRF24.h
//
//
//  Created by Andrea Vivani on 24/11/15.
//  Copyright (c) 2015 Andrea Vivani. All rights reserved.
//

#ifndef _NRF24_H_
#define _NRF24_H_

#include "stdint.h"
//======================================Parameters=============================================//
// PA Level
#define NRF24_PA_MIN   0
#define NRF24_PA_LOW   1
#define NRF24_PA_HIGH  2
#define NRF24_PA_MAX   3
// Data rate
#define NRF24_250KBPS  0
#define NRF24_1MBPS    1
#define NRF24_2MBPS    2 
// CRC Length
#define NRF24_CRC_0    0
#define NRF24_CRC_8    1
#define NRF24_CRC_16   2
// Dyn Payload
#define NRF24_EN_DYN_PYL   1
#define NRF24_DIS_DYN_PYL  0
// Auto ACK
#define NRF24_EN_AUT_ACK  1
#define NRF24_DIS_AUT_ACK  0
// Address width
#define NRF24_ADDR_1 1
#define NRF24_ADDR_2 2
#define NRF24_ADDR_3 3
#define NRF24_ADDR_4 4
#define NRF24_ADDR_5 5
// Auto-Retr Delay
#define NRF24_ARD_250  0x0
#define NRF24_ARD_500  0x1
#define NRF24_ARD_750  0x2
#define NRF24_ARD_1000 0x3
#define NRF24_ARD_1250 0x4
#define NRF24_ARD_1500 0x5
#define NRF24_ARD_1750 0x6
#define NRF24_ARD_2000 0x7
#define NRF24_ARD_2250 0x8
#define NRF24_ARD_2500 0x9
#define NRF24_ARD_2750 0xA
#define NRF24_ARD_3000 0xB
#define NRF24_ARD_3250 0xC
#define NRF24_ARD_3500 0xD
#define NRF24_ARD_3750 0xE
#define NRF24_ARD_4000 0xF
// Auto-Retr Count
#define NRF24_ARC_0  0x0
#define NRF24_ARC_1  0x1
#define NRF24_ARC_2  0x2
#define NRF24_ARC_3  0x3
#define NRF24_ARC_4  0x4
#define NRF24_ARC_5  0x5
#define NRF24_ARC_6  0x6
#define NRF24_ARC_7  0x7
#define NRF24_ARC_8  0x8
#define NRF24_ARC_9  0x9
#define NRF24_ARC_10 0xA
#define NRF24_ARC_11 0xB
#define NRF24_ARC_12 0xC
#define NRF24_ARC_13 0xD
#define NRF24_ARC_14 0xE
#define NRF24_ARC_15 0xF
// STATUS vals
// usage is_tx_sent=!(whatHappened() % NRF24_TX_SENT);
#define NRF24_TX_SENT  5
#define NRF24_MAX_RETR 3
#define NRF24_RX_AVAIL 2

    void NRF24_init(void);
    uint8_t NRF24_config(uint8_t channel, uint8_t power_level, uint8_t data_rate, uint8_t payload_size, uint8_t dyn_payload, uint8_t addr_width, uint8_t CRC_length, uint8_t auto_ack_all, uint8_t delay_retr, uint8_t count_retr);
    void NRF24_setAutoAck(uint8_t pipe, uint8_t enable);
    uint8_t NRF24_available(void);
    uint8_t NRF24_pipeAvailable(void);
    void NRF24_turnOnRadio(void);
    void NRF24_turnOffRadio(void);
    uint8_t NRF24_getStatus(void);
    uint8_t NRF24_flushTXBuffer(void);
    uint8_t NRF24_flushRXBuffer(void);
    void NRF24_maskIRQ(uint8_t tx, uint8_t fail, uint8_t rx);
    uint8_t NRF24_getChannel(void);
    uint8_t NRF24_getDynamicPayloadLength(void);
    void NRF24_setCRCLength(uint8_t length);
    uint8_t NRF24_getCRCLength(void);
    uint8_t NRF24_checkRPD(void);
    void NRF24_reUseTX(void);
    void NRF24_setPAPower(uint8_t level);
    uint8_t NRF24_getPAPower(void);
    uint8_t NRF24_setDataRate(uint8_t rate);
    uint8_t NRF24_getDataRate(void);
    void NRF24_enableAckPayload(void);
    void NRF24_enableDynamicAck(void);
    uint8_t NRF24_whatHappened(void);
    void NRF24_startTransmitter(void);
    void NRF24_startReceiver(void);
    void NRF24_openWritingPipe(const void* address);
    void NRF24_reopenWritingPipe(void);
    void NRF24_openReadingPipe(uint8_t pipe, const void* address);
    void NRF24_closeReadingPipe(uint8_t pipe);
    void NRF24_read(void* buf, uint8_t len);
    void NRF24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t data_len);
    uint8_t NRF24_write(const void* buf, uint8_t len, const uint8_t multicast, const uint8_t wait, uint32_t timeout);
    uint8_t NRF24_safeWrite(const void* buf, uint8_t len, const uint8_t multicast, const uint8_t wait, uint32_t timeout);
    void NRF24_directWrite(const void* buf, uint8_t len, const uint8_t multicast, uint8_t startTx);
    uint8_t NRF24_TXStandby(uint8_t wait_dispatch, uint32_t timeout);
    uint8_t NRF24_RXMatch(void* address, uint8_t working_byte, uint32_t delayms, uint32_t timeout); // matches the sender address, delay and timeout in ms
    void NRF24_RandTXAddr(void); // choose a random TX address
		// Ex private members
		uint8_t NRF24_readRegister(uint8_t thisRegister);
    void NRF24_readMultRegister(uint8_t* buffer, uint8_t thisRegister, uint8_t length);
    void NRF24_writeRegister(uint8_t thisRegister, const uint8_t thisValue);
    void NRF24_writeMultRegister(uint8_t thisRegister, const void* thisValue, uint8_t length);
    uint8_t NRF24_sendCMD(uint8_t cmd);
    uint8_t NRF24_write_payload(const void* buf, uint8_t data_len, const uint8_t writeType);
    uint8_t NRF24_read_payload(void* buf, uint8_t data_len);
    extern uint8_t NRF24_tx_address[5]; //TX address
#endif //_NRF24_H_


