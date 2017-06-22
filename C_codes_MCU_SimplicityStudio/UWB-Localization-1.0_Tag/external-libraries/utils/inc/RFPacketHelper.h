#ifndef UTILS_RFPACKETHELPER_H_
#define UTILS_RFPACKETHELPER_H_

#include <stdint.h>
#include <stdlib.h>

#include "em_aes.h"

#include "CRC.h"
#include "WORKMODES.h"

/*
 *
 */
void RFPacket_add_cc_length(uint8_t *buffer, uint8_t *LENGTH);

/*
 *
 */
void RFPacket_add_cc_length_DWM1000(uint8_t *buffer, uint8_t *LENGTH);

/*
 *
 */
void RFPacket_build_header(uint8_t *buffer, uint8_t *data, uint32_t *SOURCE_ID, uint32_t *DESTINATION_ID, uint8_t OPTIONS, uint8_t *LENGTH);

/*
 *
 */
void RFPacket_build_header_DWM1000(uint8_t *buffer, uint8_t *data, uint32_t *SOURCE_ID,
		uint32_t *DESTINATION_ID, uint8_t OPTIONS, uint8_t *LENGTH, uint8_t actual_packet, uint8_t total_n_packet);


/*
 *
 */
bool RFPacket_check_crc16(uint8_t *buffer);

/*
 *
 */
void RFPacket_encrypt_AES128CBC(uint8_t *buffer, uint8_t *length,
		const uint8_t *key, const uint8_t *iv);

/*
 *
 */
uint8_t RFPacket_decrypt_AES128CBC(uint8_t *buffer, const uint8_t *key,
		const uint8_t *iv);

uint8_t RFPacket_check_and_remove_header(uint8_t *data, uint32_t *SOURCE, uint32_t *DEST, uint8_t *MSG_TYPE);

#endif /* UTILS_RFPACKETHELPER_H_ */
