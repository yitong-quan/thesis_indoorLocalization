#include "WORKMODES.h"
#include "EEPROM_handler.h"

//
void MODE_broadcast_mode(uint8_t *buffer, uint8_t *length, uint8_t broadcast_mode, uint32_t SOURCE_ADDR, uint32_t DEST_ADDR, uint32_t WAKEUP_ADDR, const uint8_t *key, const uint8_t *iv) {
	uint8_t cc_data[128] = { 0 };
	*length = 0;

	#if UART_DEBUG
	char str_buffer[128] = { 0 };
	sprintf(str_buffer, "BROADCAST Mode: %d!", broadcast_mode);
	UART_WriteString(str_buffer, sizeof(str_buffer));
	memset(str_buffer, '\0', sizeof(str_buffer));
	#endif

	// React on Broadcast Number
	switch (broadcast_mode) {
	case BROADCAST_ANTI_THEFT:
		// TODO: KBS UART
		return;
	}

	// Build Packet for Base Station
	uint64_t unique_id = SYSTEM_GetUnique();
	#if REVISION == IDENTIFIER_REV_3 && KBS_UART_ENABLED
	uint16_t BAT = UART_kbs_get_bat_voltage();
	#else
	uint16_t BAT = ADC_get_bat_voltage();
	#endif

	// BROADCAST MODE
	cc_data[*length] = broadcast_mode;
	*length += 1;
	// BAT VOLTAGE
	cc_data[*length] = (BAT >> 8);
	*length += 1;
	cc_data[*length] = (BAT & 0xFF);
	*length += 1;
	// UNIQUE ID OF MCU
	cc_data[*length] = (unique_id >> 56);
	*length += 1;
	cc_data[*length] = (unique_id >> 48);
	*length += 1;
	cc_data[*length] = (unique_id >> 40);
	*length += 1;
	cc_data[*length] = (unique_id >> 32);
	*length += 1;
	cc_data[*length] = (unique_id >> 24);
	*length += 1;
	cc_data[*length] = (unique_id >> 16);
	*length += 1;
	cc_data[*length] = (unique_id >> 8);
	*length += 1;
	cc_data[*length] = (unique_id >> 0);
	*length += 1;
	// WAKEUP ADDRESS
	cc_data[*length] = (WAKEUP_ADDR >> 16);
	*length += 1;
	cc_data[*length] = (WAKEUP_ADDR >> 8);
	*length += 1;
	cc_data[*length] = (WAKEUP_ADDR);
	*length += 1;

	// Build Header
	RFPacket_build_header(buffer, cc_data, &SOURCE_ADDR, &DEST_ADDR, CC_OPT_BYTE_MSG_BROADCAST, length);

	// Add length byte for CC1101
	RFPacket_add_cc_length(buffer, length);

	// Encrypt send packet (AES128CBC)
	RFPacket_encrypt_AES128CBC(buffer, length, key, iv);
}

//
uint8_t MODE_identifier_receiving_mode(uint8_t *cc_data, uint16_t length) {

	#if UART_DEBUG
	char str_buffer[256] = { 0 };
	sprintf(str_buffer, "IDENTIFIER RECEIVER MODE!");
	UART_WriteString(str_buffer, sizeof(str_buffer));
	memset(str_buffer, '\0', sizeof(str_buffer));
	#endif

	uint8_t ret = 0xFF;
	#if REVISION == IDENTIFIER_REV_3 && KBS_UART_ENABLED

		ret = UART_kbs_transmit((char) cc_data[0], (char*) cc_data, length);
		if (ret == UART_CMD_NAK || ret == UART_CMD_NUL || ret == UART_CMD_TOUT) {
			// Simply try again
			ret = UART_kbs_transmit((char) cc_data[0], (char*) cc_data, length);
		}

	#endif

	return ret;
}

//
void MODE_identifier_status_mode(uint8_t *buffer, uint8_t *length, char *uart, uint16_t *uart_length, uint8_t OPT_BYTE, int8_t RSSI, uint32_t SOURCE_ADDR, uint32_t DEST_ADDR, const uint8_t *key, const uint8_t *iv) {

	#if UART_DEBUG
	char str_buffer[128] = { 0 };
	sprintf(str_buffer, "IDENTIFIER STATUS MODE!");
	UART_WriteString(str_buffer, sizeof(str_buffer));
	memset(str_buffer, '\0', sizeof(str_buffer));
	#endif

	uint8_t cc_data[256] = { 0 };
	*length = 0;
	uint16_t BAT = UART_kbs_get_bat_voltage();

	// AS RSSI
	cc_data[*length] = RSSI;
	*length = *length + 1;
	// BAT VOLTAGE
	cc_data[*length] = (BAT >> 8);
	*length = *length + 1;
	cc_data[*length] = (BAT & 0xFF);
	*length = *length + 1;
	// OPTION BYTE
	cc_data[*length] = OPT_BYTE;
	*length = *length + 1;
	// UART DATA LENGTH
	cc_data[*length] = *uart_length;
	*length = *length + 1;

	// UART DATA
	uint16_t i, len = *uart_length;
	for (i = 0; i < len; i++) {
		cc_data[*length] = uart[i];
		*length = *length + 1;
	}

	// Build Header
	RFPacket_build_header(buffer, cc_data, &SOURCE_ADDR, &DEST_ADDR, CC_OPT_BYTE_MSG_IDENTIFIER_TO_BASIS, length);

	// Add length byte for CC1101
	RFPacket_add_cc_length(buffer, length);

	// Encrypt send packet (AES128CBC)
	RFPacket_encrypt_AES128CBC(buffer, length, key, iv);
}

//
void MODE_table_status_mode(uint8_t *buffer, uint8_t *length, int8_t RSSI, uint32_t SOURCE_ADDR, uint32_t DEST_ADDR, const uint8_t *key, const uint8_t *iv) {

	#if UART_DEBUG
	char str_buffer[128] = { 0 };
	sprintf(str_buffer, "TABLE STATUS MODE!");
	UART_WriteString(str_buffer, sizeof(str_buffer));
	memset(str_buffer, '\0', sizeof(str_buffer));
	#endif

	uint8_t cc_data[128] = { 0 };
	*length = 0;
	uint16_t BAT = ADC_get_bat_voltage();

	// AS RSSI
	cc_data[*length] = RSSI;
	*length = *length + 1;
	// BAT VOLTAGE
	cc_data[*length] = (BAT >> 8);
	*length = *length + 1;
	cc_data[*length] = (BAT & 0xFF);
	*length = *length + 1;

	// Build Header
	RFPacket_build_header(buffer, cc_data, &SOURCE_ADDR, &DEST_ADDR, CC_OPT_BYTE_MSG_IDENTIFIER_TO_BASIS, length);

	// Add length byte for CC1101
	RFPacket_add_cc_length(buffer, length);

	// Encrypt send packet (AES128CBC)
	RFPacket_encrypt_AES128CBC(buffer, length, key, iv);
}

//
bool MODE_service_mode(uint8_t *cc_data) {
	// Check if message equals to my uinique id
	uint64_t my_unique_id = SYSTEM_GetUnique(), rec_unique_id = 0x00;
	// Read Device Unique ID
	uint8_t i;
	for (i = 0; i < 8; i++) {
		uint8_t shift = ((7-i)*8);
		rec_unique_id |= (uint64_t) cc_data[i] << shift;
	}

	if (my_unique_id != rec_unique_id) {
		return false;
	}

	// Read New ID
	uint32_t newID = (((uint32_t) cc_data[8]) << 16 | ((uint32_t) cc_data[9]) << 8 | cc_data[10]);

	// Read New WakeUp ID
	uint32_t newWakeUpID = (((uint32_t) cc_data[11]) << 16 | ((uint32_t) cc_data[12]) << 8 | cc_data[13]);

	// Read New AES Key
	uint8_t newAESkey[16] = { 0 };
	for (i = 0; i < 16; i++) {
		newAESkey[i] = cc_data[i+14];
	}

	EEPROM_write(&newID, &newWakeUpID, newAESkey);

	return true;
}
