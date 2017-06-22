#ifndef UTILS_WAKEUP_H_
#define UTILS_WAKEUP_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <em_gpio.h>
#include "config.h"

// Bits to transmit 4 periods of LF signal; ATTENTION: inverted in comparision to old versions due to inverted PA table
#define LF_LOGIC_ONE    	0xAA
#define LF_LOGIC_ZERO    	0x00

// Length of AS3933 bit as multiples of 4 LF carrier periods (use multiples of 4 in order to comply with the sending possiblities of the CC1101)
// LENGTH_WU_BIT_CC = LENGTH_WU_BIT/4 with LENGTH_WU_BIT being the number of LF periods to cover one AS3933 bit
// LENGTH_WU_BIT = 8 @ 19.2 kHz; LENGTH_WU_BIT = 16 @ 125 kHz;
#if 	LF_FREQ == 19200
	#define LENGTH_WU_BIT_CC	2 	//old: 1
#elif 	LF_FREQ == 125000
	#define LENGTH_WU_BIT_CC	4
#endif

// Preamble length (in AS3933 manchester symbols)
#define LENGTH_PREAMBLE			6

// Length of LF carrier burst as multiples of 4 LF carrier periods (use multiples of 4 in order to comply with the sending possiblities of the CC1101)
// A LF carrier burst of approx. 3.5 ms is highly recommended, use LENGTH_CB_CC = int(3.5/1000/(LENGTH_WU_BIT/LF_FREQ))+1
#if 	LF_FREQ == 19200
	#define LENGTH_CB_CC		17
#elif 	LF_FREQ == 125000
	#define LENGTH_CB_CC		28
#endif

/*
 *
 */

void wakeup_settings(uint16_t options);

uint16_t wakeup_build_packet_8bit(uint8_t address, uint8_t *buffer,
		uint8_t *payload, uint16_t payload_length);

uint16_t wakeup_build_packet_16bit(uint16_t address, uint8_t *buffer,
		uint8_t *payload, uint16_t payload_length);

uint16_t broadcast_build_packet(uint8_t *buffer, uint8_t *payload,
		uint16_t payload_length);

void wakeup_add_payload(uint16_t *position, uint8_t *buffer, uint8_t *payload,
		uint16_t payload_length);

void wakeup_show_on_pin(uint8_t *buffer, uint16_t length,
		GPIO_Port_TypeDef port, uint8_t pin);

// Help functions
uint8_t swapBits(uint8_t data);
uint8_t evenParityBit(uint8_t *data);
uint8_t oddParityBit(uint8_t *data);

#endif /* UTILS_WAKEUP_H_ */
