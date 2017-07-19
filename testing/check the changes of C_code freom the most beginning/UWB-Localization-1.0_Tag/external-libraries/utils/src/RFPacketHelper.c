#include "RFPACKETHELPER.h"

//
void RFPacket_add_cc_length(uint8_t *buffer, uint8_t *LENGTH) {
	uint8_t tmp[256] = { 0 };

	tmp[0] = *LENGTH + 1;

	uint16_t i;
	for (i = 0; i < *LENGTH; i++) {
		tmp[i+1] = buffer[i];
	}

	for (i = 0; i < *LENGTH + 1; i++) {
		buffer[i] = tmp[i];
	}
	*LENGTH = *LENGTH + 1;
}

void RFPacket_add_cc_length_DWM1000(uint8_t *buffer, uint8_t *LENGTH) {
	uint8_t tmp[256] = { 0 };

	tmp[0] = *LENGTH + 1;

	uint16_t i;
	for (i = 0; i < *LENGTH; i++) {
		tmp[i+1] = buffer[i];
	}

	for (i = 0; i < *LENGTH + 1; i++) {
		buffer[i] = tmp[i];
	}
	*LENGTH = *LENGTH + 1;
}

void RFPacket_build_header(uint8_t *buffer, uint8_t *data, uint32_t *SOURCE_ID, uint32_t *DESTINATION_ID, uint8_t OPTIONS, uint8_t *LENGTH) {
	uint16_t crc = 0;
	buffer[0] = *SOURCE_ID >> 16;
	crc = AddToCrc16(crc, buffer[0]);
	buffer[1] = *SOURCE_ID >> 8;
	crc = AddToCrc16(crc, buffer[1]);
	buffer[2] = *SOURCE_ID;
	crc = AddToCrc16(crc, buffer[2]);
	buffer[3] = *DESTINATION_ID >> 16;
	crc = AddToCrc16(crc, buffer[3]);
	buffer[4] = *DESTINATION_ID >> 8;
	crc = AddToCrc16(crc, buffer[4]);
	buffer[5] = *DESTINATION_ID;
	crc = AddToCrc16(crc, buffer[5]);
	buffer[6] = OPTIONS;  // Options
	crc = AddToCrc16(crc, buffer[6]);
	buffer[7] = *LENGTH;  // Data length
	crc = AddToCrc16(crc, buffer[7]);
	buffer[8] = 0;  // CRC high Byte
	crc = AddToCrc16(crc, buffer[8]);
	buffer[9] = 0;  // CRC low Byte
	crc = AddToCrc16(crc, buffer[9]);

	// Copy Data
	uint16_t i;
	for (i = 0; i < *LENGTH; i++) {
		buffer[i+10] = data[i];
		crc = AddToCrc16(crc, data[i]);
	}

	// Add Checksum
	buffer[8] = (crc >> 8);
	buffer[9] = (crc & 0xFF);

	// Refresh Length
	*LENGTH += 10;
}

void RFPacket_build_header_DWM1000(uint8_t *buffer, uint8_t *data, uint32_t *SOURCE_ID,
		uint32_t *DESTINATION_ID, uint8_t OPTIONS, uint8_t *LENGTH, uint8_t actual_packet, uint8_t total_n_packet) {
	uint16_t crc = 0;
	buffer[0] = *SOURCE_ID >> 16;
	crc = AddToCrc16(crc, buffer[0]);
	buffer[1] = *SOURCE_ID >> 8;
	crc = AddToCrc16(crc, buffer[1]);
	buffer[2] = *SOURCE_ID;
	crc = AddToCrc16(crc, buffer[2]);
	buffer[3] = *DESTINATION_ID >> 16;
	crc = AddToCrc16(crc, buffer[3]);
	buffer[4] = *DESTINATION_ID >> 8;
	crc = AddToCrc16(crc, buffer[4]);
	buffer[5] = *DESTINATION_ID;
	crc = AddToCrc16(crc, buffer[5]);
	buffer[6] = OPTIONS;  // Options
	crc = AddToCrc16(crc, buffer[6]);
	buffer[7] = *LENGTH;  // Data length
	crc = AddToCrc16(crc, buffer[7]);

	buffer[8] = actual_packet;
	crc = AddToCrc16(crc, buffer[8]);

	buffer[9] = total_n_packet;
	crc = AddToCrc16(crc, buffer[9]);

	buffer[10] = 0;  // CRC high Byte
	crc = AddToCrc16(crc, buffer[10]);
	buffer[11] = 0;  // CRC low Byte
	crc = AddToCrc16(crc, buffer[11]);

	// Copy Data
	uint16_t i;
	for (i = 0; i < *LENGTH; i++) {
		buffer[i+12] = data[i];
		crc = AddToCrc16(crc, data[i]);
	}

	// Add Checksum
	buffer[10] = (crc >> 8);
	buffer[11] = (crc & 0xFF);

	// Refresh Length
	*LENGTH += 12;
}



//
bool RFPacket_check_crc16(uint8_t *buffer) {
	bool checksum_ok = false;
	uint8_t len = 10 + buffer[7];
	if (len < 255) {
		uint16_t chksum_received = (((uint16_t) buffer[8]) << 8 | buffer[9]);
		buffer[8] = 0x00;
		buffer[9] = 0x00;
		uint16_t chksum_computed = CalcBlockCrc(buffer, len);
		if (chksum_computed == chksum_received) {
			checksum_ok = true;
		}

		buffer[8] = chksum_received >> 8;
		buffer[9] = chksum_received;
	}

	return checksum_ok;
}

//
void RFPacket_encrypt_AES128CBC(uint8_t *buffer, uint8_t *length,
		const uint8_t *key, const uint8_t *iv) {
	*length = ((*length - 1) | 15) + 1;  // Must be a multiple of 16

	uint8_t AESBuffer[*length];
	memset(AESBuffer, 0x00, *length);

	uint8_t i;
	for (i = 0; i < *length; i++) {
		AESBuffer[i] = buffer[i + 1];
	}

	/* Encrypt data in AES_128 CBC */
	AES_CBC128(AESBuffer, AESBuffer, *length, key, iv, true);

	memset(buffer, 0x00, sizeof(buffer));

	// Copy AES buffer to send buffer and add packet length
	buffer[0] = *length + 1;
	*length += 1;
	for (i = 1; i <= *length; i++) {
		buffer[i] = AESBuffer[i - 1];
	}
}

//
uint8_t RFPacket_decrypt_AES128CBC(uint8_t *buffer, const uint8_t *key, const uint8_t *iv) {
	// Find length of buffer
	uint8_t len = 255;
	while (buffer[len] == 0 && len > 0) {
		len--;
	}

	// Decrypt Buffer
	AES_CBC128(buffer, buffer, len, key, iv, false);

	return len;
}

uint8_t RFPacket_check_and_remove_header(uint8_t *data, uint32_t *SOURCE, uint32_t *DEST, uint8_t *MSG_TYPE) {
	// Read source address
	uint32_t source_address = (((uint32_t) data[0]) << 16	| ((uint32_t) data[1]) << 8 | data[2]);
	if (*SOURCE == 0x00) {  // If Source is empty, return source address
		*SOURCE = source_address;
	} else { // If Source is NOT empty, compare it
		if (source_address != *SOURCE) {
			/*
            for(int nn = 0; nn< 2; nn++){
                    LED_setLED(COL_RED);
                    RTC_delay_ms(20);
                    LED_clearLED();
                    RTC_delay_ms(20);//++++++++++++++++++++++!!!!!!!!!!!!!!!!!!!!!!!!!!Yitong
            }
			*/
			return 0;
		}
	}

	// Read destination address
	uint32_t destination_addr = (((uint32_t) data[3]) << 16 | ((uint32_t) data[4]) << 8 | data[5]);
	if (*DEST == 0x00) {  // If Destination is empty, return Destination address
		*DEST = destination_addr;
	} else {
		if (destination_addr != *DEST) {
			/*
            for(int nn = 0; nn< 2; nn++){
                    LED_setLED(COL_RED);
                    RTC_delay_ms(15);
                    LED_clearLED(); //++++++++++++++++++++++++++!!!!!!!!!!!!!!!!!!!!!!Yitong
                    RTC_delay_ms(15);
            }
			*/
			return 0;
		}
	}

	// Check Message Type
	uint8_t message_type = data[6];
	if (*MSG_TYPE == 0x00) {  // If Destination is empty, return Destination address
		//RTC_delay_ms(2);
		*MSG_TYPE = message_type;
	}else {
		if (message_type != *MSG_TYPE) {
			/*
            for(int nn = 0; nn< 2; nn++){
                    LED_setLED(COL_RED);
                    RTC_delay_ms(25);
                    LED_clearLED();
                    RTC_delay_ms(25);//++++++++++++++++++++++++++!!!!!!!!!!!!!!!!!!!!!!Yitong
            }
			*/
			return 0;
		}
	}

	// Check CRC
	if (!RFPacket_check_crc16(data)) {
		/*
        for(int nn = 0; nn< 2; nn++){
                LED_setLED(COL_RED);
                RTC_delay_ms(30);
                LED_clearLED();
                RTC_delay_ms(30);
        }
		*/
        return 0;
	}

	if (message_type == CC_OPT_BYTE_ACK) {
		return 0x06; // ASCII ACK SIGN
	} else {
		// Remove Header / Extract Data
		uint16_t data_length = data[7], i = 0;
		//uint32_t data_length = (uint32_t)data[7], i = 0;
		for (i=0; i < data_length + 10; i++) {
			if (i < data_length) {
				data[i] = data[i+10];
			} else {
				data[i] = 0x00;
			}
		}
		return data_length;
	}
}
