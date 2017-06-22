#ifndef UTILS_RADIO_H_
#define UTILS_RADIO_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "em_cmu.h"
#include "em_timer.h"
#include <em_gpio.h>

#include "CC1101.h"
#include "RADIO_CONFIG.h"
#include "WAKEUP.h"
#include "CRC.h"
#include "TIMER.h"
#include "config.h"
#include "RFPacketHelper.h"

// DEBUG:
#include "UART.h"

/* T.Kumberg 04.09.2014:
 - Inserted variables in struct to send wakeup packets
 - Increased size of num_bytes and num_bytes_left to uint16_t to be
 able to send more than 256 bytes
 - Inserted function radio_send_inf_wakeup_packet(...)
 */

#define RADIO_MAX_PAYLOAD_LENGTH	254

struct radio_transfer {
	/* Determines the transfer mode. */bool receive_mode;

	/* Number of bytes transferred (sent / received). */
	uint16_t num_bytes;

	/* Number of byte to transfer. */
	uint16_t num_bytes_left;

	/* following variables are used to send wakeups in infinite mode */
	/* Number of packets to transmit */
	uint8_t num_last_bytes;

	/* true to sent wakeup packet in infinite mode */bool send_inf_wakeup_packet;

	/* TODO: Comment: Determines if the length byte is received yet. */bool length_received;

	uint8_t *buffer;

	/* TODO: Comment: Determines if the transfer completed. */bool finished;

	/* TODO: Comment: Determines if the transfer timed out. */bool timedout;

	uint8_t length;

	/* Status of received data (LQI and RSSI). */
	uint8_t status[2];

	/* Timeout timer. */
	TIMER_TypeDef *timer;
};

enum {
	RADIO_OK = 0, RADIO_ERR_CRC = -1, RADIO_ERR_TIMEOUT = -2
};

void radio_init(struct radio_transfer *xfer, TIMER_TypeDef *timer,
		CMU_Clock_TypeDef timer_clock, IRQn_Type timer_irq);

void radio_handle_transfer_threshold(struct radio_transfer *xfer);

void radio_handle_transfer_end(struct radio_transfer *xfer);

void radio_handle_timeout(struct radio_transfer *xfer);

uint8_t wakeup_build_packet(uint8_t *buffer, const uint8_t *payload,
		uint8_t length);

bool radio_send_packet_use_CSMA_CA(struct radio_transfer *xfer, uint8_t *buffer,
		uint8_t length);

void radio_send_packet(struct radio_transfer *xfer, uint8_t *buffer,
		uint8_t length);

void radio_send_inf_wakeup_packet(struct radio_transfer *xfer, uint8_t *buffer,
		uint16_t length);

int radio_receive_packet(struct radio_transfer *xfer, uint8_t *buffer,
		uint8_t *length, uint16_t timeout_ms);

int radio_receive_packet_DWM1000(struct radio_transfer *xfer, uint8_t *buffer,
		uint16_t *length, uint16_t timeout_ms);

void radio_receive_packet_polling_init(struct radio_transfer *xfer,
		uint8_t *buffer);

int radio_receive_packet_polling(struct radio_transfer *xfer, uint8_t *length);

bool radio_channel_busy(void);

void radio_wait(uint16_t uDelay);

void cc1101_change_config_to(uint8_t configuration, uint8_t * patable);

void cc1101_power_down(void);

void cc1101_send_wakeup_packet_8bit_addr(struct radio_transfer *xfer,
		uint8_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t options, uint16_t measurements);

void cc1101_send_wakeup_packet_16bit_addr(struct radio_transfer *xfer,
		uint16_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t options, uint16_t measurements);

void cc1101_send_wakeup_packet_24bit_addr(struct radio_transfer *xfer,
		uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t options, uint16_t measurements);

void cc1101_send_broadcast_packet(struct radio_transfer *xfer,
		uint8_t broadcastData);

void cc1101_ack_reception(struct radio_transfer *xfer, uint32_t *SOURCE_ID,
		uint32_t *DESTINATION_ID, const uint8_t *key, const uint8_t *iv);

bool cc1101_check_ack(struct radio_transfer *xfer, uint32_t *SOURCE_ID,
		uint32_t *DESTINATION_ID, const uint8_t *key, const uint8_t *iv,
		uint16_t timeout_ms);

void cc1101_set_patable(uint8_t[8]);

int8_t cc1101_read_rssi(void);

#endif /* UTILS_RADIO_H_ */
