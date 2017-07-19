#ifndef UTILS_WORKMODES_H_
#define UTILS_WORKMODES_H_

#include <stdint.h>
#include <stdio.h>

#include "em_aes.h"
#include "em_system.h"

#include "UART.h"
#include "DMA_LEUART.h"
#include "CRC.h"
#include "RTC.h"
#include "MISC.h"
#include "RFPACKETHELPER.h"

// CC OPTION BYTE
#define CC_OPT_BYTE_MSG_SERVICE				 	0x6B
#define CC_OPT_BYTE_MSG_BROADCAST			 	0x6B
#define CC_OPT_BYTE_MSG_BASIS_TO_TERMINAL	 	0x6A
#define CC_OPT_BYTE_MSG_TERMINAL_TO_BASIS	 	0x69
#define CC_OPT_BYTE_MSG_IDENTIFIER_TO_BASIS 	0x68
#define CC_OPT_BYTE_MSG_BASIS_TO_IDENTIFIER 	0x67
#define CC_OPT_BYTE_MSG_TABLE_TO_IDENTIFIER 	0x66
#define CC_OPT_BYTE_MSG_IDENTIFIER_TO_TABLE 	0x65
#define CC_OPT_BYTE_DAT_IDENTIFIER_TO_BASIS 	0x64
#define CC_OPT_BYTE_DAT_BASIS_TO_IDENTIFIER 	0x63
#define CC_OPT_BYTE_DAT_TABLE_TO_IDENTIFIER 	0x62
#define CC_OPT_BYTE_DAT_IDENTIFIER_TO_TABLE 	0x61

#define CC_OPT_BYTE_ACK						 	0x40


// AS OPTION BYTE MODES
#define AS_OPT_BYTE_NORMAL_MODE					0xAA
#define AS_OPT_BYTE_DATA_MODE					0x11
#define AS_OPT_BYTE_STATUS_MODE					0x22
#define AS_OPT_BYTE_SERVICE_MODE				0x33

// GET_STATE_OPT_BYTE
#define STATE_OPT_BYTE_DLE_SUCCESS				(1 << 0)

// BROADCAST MODES
#define BROADCAST_ANTI_THEFT					0xAA
#define BROADCAST_SERVICE_SETTINGS				0xC1
#define BROADCAST_SERVICE_GET_STATE				0xC2

// GENERAL BROADCAST BASE ADDRESS
#define GENERAL_BROADCAST_BASE_ADDR				0xC1C1C1

//
void MODE_broadcast_mode(uint8_t *buffer, uint8_t *length, uint8_t broadcast_mode, uint32_t SOURCE_ADDR, uint32_t DEST_ADDR, uint32_t WAKEUP_ADDR, const uint8_t *key, const uint8_t *iv);

//
uint8_t MODE_identifier_receiving_mode(uint8_t *cc_data, uint16_t length);

//
void MODE_identifier_status_mode(uint8_t *buffer, uint8_t *length, char *uart, uint16_t *uart_length, uint8_t OPT_BYTE, int8_t RSSI, uint32_t SOURCE_ADDR, uint32_t DEST_ADDR, const uint8_t *key, const uint8_t *iv);

//
void MODE_table_status_mode(uint8_t *buffer, uint8_t *length, int8_t RSSI, uint32_t SOURCE_ADDR, uint32_t DEST_ADDR, const uint8_t *key, const uint8_t *iv);

//
bool MODE_service_mode(uint8_t *cc_data);

#endif /* UTILS_WORKMODES_H_ */
