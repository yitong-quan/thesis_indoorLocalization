/*
 * SpectralTest.c
 *
 *  Created on: 17.12.2015
 *      Author: Sebastian
 */
#include "TestModes.h"
#include "math.h"
#include "enum.h"

extern uint8_t POLL_message[];							// Poll message
extern uint8_t RESP_message[];							// Response message
extern uint8_t FINAL_message[];   						// Final message

extern enum states_localization localization;

uint32_t poll_1_tx_timestamp, poll_1_rx_timestamp, resp_1_tx_timestamp, resp_1_rx_timestamp, final_1_tx_timestamp, final_1_rx_timestamp;
uint64_t poll_tx_timestamp, poll_rx_timestamp, resp_tx_timestamp, resp_rx_timestamp, final_tx_timestamp, final_rx_timestamp;

#if (REVISION == IDENTIFIER_REV_1) || (REVISION == IDENTIFIER_REV_2) || (REVISION == IDENTIFIER_REV_3) || (REVISION == TABLE_REV_1)

void TESTMODE_enter(struct radio_transfer *xfer) {

	/* PA table settings, see config.h */
	uint8_t paTableWakeUp[8] = PA_TABLE_HIGH_POWER_REDUCED_IF;
	uint8_t paTableData[8] = PA_FSK;

	uint8_t AES_decryption_key[16];  										// AES Decryption Key
	uint8_t AES_encryption_key[] = AES_ENCRYPTION_KEY; 						// AES Encryption Key 128 bit
	uint8_t AES_decryption_public_key[16];  								// AES Decryption Key
	uint8_t AES_encryption_public_key[] = AES_ENCRYPTION_PUBLIC_KEY; 		// AES Encryption Key 128 bit
	const uint8_t AES_initVector[] = AES_INIT_VECTOR;  						// Initialization vector used during CBC
	AES_DecryptKey128(AES_decryption_key, AES_encryption_key);
	AES_DecryptKey128(AES_decryption_public_key, AES_encryption_public_key);


	// Blink LED
	LED_setLED(COL_GREEN);
	RTC_delay_ms(50);
	LED_clearLED();

	#if	MODE == TEST_WAKE_UP_TRANSMITTER
		wakeUpTestTransmit(xfer, CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);
	#elif	MODE == TEST_WAKE_UP_RECEIVER
		wakeUpTestReceive();
	#elif	MODE == TEST_WAKE_UP_SENSITIVITY
		wakeUpTestSensitivity();
	#elif	MODE == TEST_SPECTRAL_ANALYSIS
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		spectralTestEN300220(xfer, CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);
	#elif	MODE == TEST_SPECTRAL_ANALYSIS_HF_CARRIER
		spectralTestHFCarrier(xfer, CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);
	#elif	MODE == TEST_AS_OPTIMIZATION_TRANSMITTER
		AS_optimization_transmitter(xfer, CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp, 1000, false, false);
	#elif	MODE == TEST_AS_OPTIMIZATION_RECEIVER
		AS_optimization_receiver(xfer, false);
	#elif	MODE == TEST_AS_OPTIMIZATION_BASE
		AS_optimization_basestation(xfer);
	#elif	MODE == TEST_WATCHDOG_FUNCTION
		TEST_WDOG();
	#elif	MODE == TEST_BROADCAST_TRANSMITTER
		testBroadcastTransmitter(xfer, CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);
	#elif	MODE == TEST_BROADCAST_RECEIVER
		testBroadcastReceiver();
	#elif	MODE == TEST_DATA_TRANSMITTER
		TEST_data_transmitter(xfer, AES_encryption_key, AES_initVector);
	#elif	MODE == TEST_DATA_RECEIVER
		TEST_data_receiver(xfer, AES_decryption_key, AES_initVector);
	#elif	MODE == TEST_PROGRAMMER
		TEST_programmer(xfer, AES_encryption_public_key, AES_decryption_public_key, AES_initVector);
	#elif   MODE == TEST_DWM1000_INITIATOR_IRQ
		TEST_DWM_UWB_initiator_interrupt();
	#elif 	MODE == TEST_DWM1000_RESPONDER_IRQ
		TEST_DWM_UWB_responder_interrupt();
	#elif	MODE == TEST_DWM1000_NODE_SLEEP
		TEST_DWM_UWB_NODE_SLEEP();
	#elif	MODE == TEST_DWM1000_TAG_SLEEP
		TEST_DWM_TAG_SLEEP();
	#elif	MODE == TEST_DWM1000_TAG_BASIS_STATION
		TEST_DWM_TAG_TO_BASIS();
	#endif
}

/* 1. SPECTRAL TESTS */

// Starts to transmit the pattern required by ETSI EN 300 220:
// WARNING: The function will transmit infinitely!
void spectralTestEN300220(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	while(1) {
		cc1101_change_config_to(config, powerConfig);
		radio_send_inf_wakeup_packet(xfer, testPattern, 511);
	}

}

// Starts to transmit the a sequence of alternating 0 and 1 bits.
// WARNING: The function will transmit infinitely!
void spectralTestPreamble(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	cc1101_change_config_to(config, powerConfig);

	while(1) {
		radio_send_inf_wakeup_packet(xfer, testPattern2, 511);
	}

}

// Starts to transmit a wake-up sequence with address 0xCC and ID 0x00112511
// WARNING: The function will transmit infinitely!
void spectralTestWakeUp(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	cc1101_change_config_to(config, powerConfig);
	while(1) {
		cc1101_send_wakeup_packet_8bit_addr(xfer, 0xCC, 0x00112511, TEST_WAKE_UP_OPTION, 25);
	}

}

// Starts to transmit a pure carrier, powerConfig argument without use:
// WARNING: The function will transmit infinitely!
void spectralTestCarrier(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	cc1101_change_config_to(config, carrierPAtable);

	while(1) {
		radio_send_inf_wakeup_packet(xfer, testPattern2, 511);
	}
}


/* 2. WAKE-UP TESTS */

void wakeUpTestTransmit(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	while(1) {

		cc1101_change_config_to(config, powerConfig);

		#if		WU_ADDR_LENGTH == USE_8_BIT
			cc1101_send_wakeup_packet_8bit_addr(xfer, TEST_WAKE_UP_ID, 0x00112511, TEST_WAKE_UP_OPTION, 0);
		#elif	WU_ADDR_LENGTH == USE_16_BIT
			cc1101_send_wakeup_packet_16bit_addr(xfer, TEST_WAKE_UP_ID, 0x00112511, TEST_WAKE_UP_OPTION, 0);
		#elif	WU_ADDR_LENGTH == USE_24_BIT
			cc1101_send_wakeup_packet_24bit_addr(xfer, TEST_WAKE_UP_ID, 0x00112511, TEST_WAKE_UP_OPTION, 0);
		#endif

		LED_setLED(COL_RED);
		RTC_delay_ms(5);
		LED_clearLED();
		RTC_delay_ms(TEST_WU_DELAY);
	}

}

void wakeUpTestReceive() {

	while(1) {
		// Set CC1101 RF Module to WakeUp Configuration and power down
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_power_down();

		// Enable AS3933 WakeUp by Address
		#if		WU_ADDR_LENGTH == USE_8_BIT
			AS3933_enable_wakeup_by_8bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
		#elif	WU_ADDR_LENGTH == USE_16_BIT
			AS3933_enable_wakeup_by_16bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
		#elif	WU_ADDR_LENGTH == USE_24_BIT
			AS3933_enable_wakeup_by_24bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
		#endif

		// Switch antenna to WakeUp Receiver
		ANTENNA_switch(ANTENNA_DEST_WAKEUPRF);

		// Now enter SleepMode!!
		while (!WAKEUP_received) {
			__disable_irq();
			//EMU_EnterEM3(true);  // Sleep mode
			EMU_EnterEM1();  //Yitong
			__enable_irq();
		}

		// Awake
		LED_setLED(COL_RED);
		RTC_delay_ms(10);
		LED_clearLED();
	}

}

void wakeUpTestSensitivity() {

	while(1) {
		// Set CC1101 RF Module to WakeUp Configuration and power down
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_power_down();

		// Enable AS3933 WakeUp by Frequency Detection (Pattern correlation disabled)
		AS3933_enable_wakeup_by_frequency_detection(AS3933_SPI_BAUDRATE);

		// Switch antenna to WakeUp Receiver
		ANTENNA_switch(ANTENNA_DEST_WAKEUPRF);

		// Now enter SleepMode!!
		while (!WAKEUP_received) {
			__disable_irq();
			//EMU_EnterEM3(true);  // Sleep mode
			EMU_EnterEM1();  //Yitong
			__enable_irq();
		}

		// Awake
		LED_setLED(COL_RED);
		RTC_delay_ms(5);
		LED_clearLED();
	}

}

// Starts to transmit a pure UHF carrier:
// WARNING: The function will transmit infinitely!
void spectralTestHFCarrier(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	cc1101_change_config_to(config, carrierPAtable);

	cc1101_spi_strobe(CC1101_STX);
	while(1);
}

//
void AS_optimization_transmitter(struct radio_transfer *xfer, uint8_t WU_config,
		uint8_t * WU_powerConfig, uint16_t max_num_of_packets, bool use_wakeup, bool big_data) {

	int8_t * rssi_samples;
	rssi_samples = malloc(max_num_of_packets);

	while (1) {

		char str_buffer[128] = { 0 };
		uint8_t paTableData[8] = PA_FSK_LOW;

		uint16_t num_of_ack = 0, num_of_packets = 0, index = 0;

		memset(rssi_samples, 0x00, max_num_of_packets);

		ANTENNA_switch(ANTENNA_DEST_CC1101);

		while(num_of_packets < max_num_of_packets) {

			uint8_t CC1101_receive[64] = { 0 };
			uint8_t len = 0;

			if (use_wakeup) {
				cc1101_spi_init(CC1101_SPI_BAUDRATE);
				cc1101_change_config_to(WU_config, WU_powerConfig);

				#if		WU_ADDR_LENGTH == USE_8_BIT
					cc1101_send_wakeup_packet_8bit_addr(xfer, TEST_WAKE_UP_ID, 0x00112511, TEST_WAKE_UP_OPTION, 25);
				#elif	WU_ADDR_LENGTH == USE_16_BIT
					cc1101_send_wakeup_packet_16bit_addr(xfer, TEST_WAKE_UP_ID, 0x00112511, TEST_WAKE_UP_OPTION, 25);
				#elif	WU_ADDR_LENGTH == USE_24_BIT
					cc1101_send_wakeup_packet_24bit_addr(xfer, TEST_WAKE_UP_ID, 0x00112511, TEST_WAKE_UP_OPTION, 25);
				#endif
			} else {
				ANTENNA_switch(ANTENNA_DEST_CC1101);
				cc1101_spi_init(CC1101_SPI_BAUDRATE);
				cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

				uint8_t send_buf[16] = {0};
				send_buf[0] = 6;
				send_buf[1] = 'S';
				send_buf[2] = 'T';
				send_buf[3] = 'A';
				send_buf[4] = 'R';
				send_buf[5] = 'T';
				#if USE_CSMA_CA
					radio_send_packet_use_CSMA_CA(xfer, send_buf, 6);
				#else
					radio_send_packet(xfer, send_buf, 6);
				#endif
			}

			num_of_packets++;

			// Set CC1101 RF Module to Data Communication Configuration
			cc1101_spi_init(CC1101_SPI_BAUDRATE);
			cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

			// Receive ACK from Tables
			RTC_start(100);
			radio_receive_packet(xfer, CC1101_receive, &len, 20);
			while (!(CC1101_receive[0] == 'A' && CC1101_receive[1] == 'C' && CC1101_receive[2] == 'K') && !RTC_TIMEOUT) {
				memset(CC1101_receive, 0x00, sizeof(CC1101_receive));
				radio_receive_packet(xfer, CC1101_receive, &len, 20);
			}

			if(!RTC_TIMEOUT) {
				num_of_ack++;
				rssi_samples[index++] = cc1101_read_rssi();
				if(!big_data) {
					sprintf(str_buffer, "Packet %d -> RSSI %d\r\n", num_of_packets, rssi_samples[index-1]);
					UART_WriteString(str_buffer, sizeof(str_buffer));
					memset(str_buffer, '\0', sizeof(str_buffer));
				}
			} else {
				//rssi_samples[index++] = 0;
				if(!big_data) {
					sprintf(str_buffer, "Packet %d -> NOT ACK!\r\n", num_of_packets);
					UART_WriteString(str_buffer, sizeof(str_buffer));
					memset(str_buffer, '\0', sizeof(str_buffer));
				}
			}

			LED_setLED(COL_RED);
			TIMER_delay_ms(TIMER0, 1, false);
			LED_clearLED();
		}

		// MATH
		int32_t rssi = array_sum(rssi_samples, num_of_ack);
		uint16_t errors = num_of_packets - num_of_ack;
		float avg_rssi = (rssi * 1.0) /(num_of_ack * 1.0);
		float std_dev = array_std_deviation(rssi_samples, avg_rssi, num_of_ack);
		float dist = 2.74721 / (pow(10, (avg_rssi - 1.9) / 20.0));
		//uint16_t dist = 0;// lut_distance2[(int) (-avg_rssi)];

		// PRINT
		char tmp1[20] = {0};
		char tmp2[20] = {0};
		char tmp3[20] = {0};
		ftoa(tmp1, avg_rssi);
		ftoa(tmp2, dist);
		ftoa(tmp3, std_dev);
		sprintf(str_buffer, "\r\nPackets: %d ACK: %d Errors: %d RSSI: %s Distance: %s STD_DEV: %s\r\n\r\n", num_of_packets, num_of_ack, errors, tmp1, tmp2, tmp3);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));

		/* Initial Switch antenna to CC1101 RF Module */
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

		uint8_t CC1101_transmit[64] = { 0 };
		uint8_t idx = 1;
		CC1101_transmit[idx++] = 'F';
		CC1101_transmit[idx++] = 'I';
		CC1101_transmit[idx++] = 'N';
		CC1101_transmit[idx++] = (num_of_packets >> 8);
		CC1101_transmit[idx++] = num_of_packets & 0xFF;
		CC1101_transmit[idx++] = (num_of_ack >> 8);
		CC1101_transmit[idx++] = num_of_ack & 0xFF;
		CC1101_transmit[idx++] = (errors >> 8);
		CC1101_transmit[idx++] = errors & 0xFF;

		uint8_t *ptr1 = (uint8_t *) &avg_rssi;
		uint8_t *ptr2 = (uint8_t *) &dist;
		uint8_t *ptr3 = (uint8_t *) &std_dev;

		// transmit avg_rssi
		CC1101_transmit[idx++] = ptr1[0];
		CC1101_transmit[idx++] = ptr1[1];
		CC1101_transmit[idx++] = ptr1[2];
		CC1101_transmit[idx++] = ptr1[3];

		// transmit dist
		CC1101_transmit[idx++] = ptr2[0];
		CC1101_transmit[idx++] = ptr2[1];
		CC1101_transmit[idx++] = ptr2[2];
		CC1101_transmit[idx++] = ptr2[3];

		// transmit std_dev
		CC1101_transmit[idx++] = ptr3[0];
		CC1101_transmit[idx++] = ptr3[1];
		CC1101_transmit[idx++] = ptr3[2];
		CC1101_transmit[idx++] = ptr3[3];

		CC1101_transmit[0] = idx;

		RTC_delay_ms(1000);

		#if USE_CSMA_CA
			radio_send_packet_use_CSMA_CA(xfer, CC1101_transmit, idx);
		#else
			radio_send_packet(xfer, CC1101_transmit, idx);
		#endif

		if(big_data) {
			UART_WriteString("All samples:\r\n\r\nlist={", sizeof("All samples:\r\n\r\nlist={"));
			uint16_t j;
			for (j = 0; j < max_num_of_packets; j++) {
				if (rssi_samples[j] != 0) {
					if(j == max_num_of_packets - 1) {
						sprintf(str_buffer, "%d", rssi_samples[j]);
						UART_WriteString(str_buffer, sizeof(str_buffer));
						memset(str_buffer, '\0', sizeof(str_buffer));
					} else {
						sprintf(str_buffer, "%d, ", rssi_samples[j]);
						UART_WriteString(str_buffer, sizeof(str_buffer));
						memset(str_buffer, '\0', sizeof(str_buffer));
					}
				}
			}
			UART_WriteString(" };", sizeof(" };"));
		}

		RTC_delay_ms(2000);
	}
}

// Help function
int32_t array_sum(int8_t * array, uint16_t size) {
	uint16_t i;
	int32_t sum = 0;
	for (i = 0; i < size; i++) {
		sum += array[i];
	}
	return sum;
}

// Help function
float array_std_deviation(int8_t * array, float avg, uint16_t size) {
	uint16_t i;
	float sum = 0;
	for (i = 1; i < size; i++) {
		float delta = (array[i] - avg);
		sum += pow(delta, 2);
	}

	float under_the_root = 1/(1.0*size-1) * sum;
	float result = sqrt(under_the_root);
	return result;
}

// Help function for converting a float number to ascii string
void ftoa(char* str, float fNum) {
	int32_t wholeNum = fNum;
	uint32_t decimalNum = abs(((fNum * 1000) - (wholeNum * 1000)));
	sprintf(str,"%d.%d", (int) wholeNum, (int) decimalNum);
}

//
void AS_optimization_receiver(struct radio_transfer *xfer, bool use_wakeup) {
	uint8_t paTableData[8] = PA_FSK_LOW;

	while(1) {

		uint8_t CC1101_receive[64] = { 0 };
		uint8_t len = 0;

		if (use_wakeup) {
			// RF module power down
			cc1101_spi_init(CC1101_SPI_BAUDRATE);
			cc1101_power_down();

			// Enable AS3933 WakeUp by Address
			#if		WU_ADDR_LENGTH == USE_8_BIT
				AS3933_enable_wakeup_by_8bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
			#elif	WU_ADDR_LENGTH == USE_16_BIT
				AS3933_enable_wakeup_by_16bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
			#elif	WU_ADDR_LENGTH == USE_24_BIT
				AS3933_enable_wakeup_by_24bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
			#endif

			// Switch antenna to WakeUp Receiver
			ANTENNA_switch(ANTENNA_DEST_WAKEUPRF);

			// Now enter SleepMode!!
			while (!WAKEUP_received) {
				__disable_irq();
				EMU_EnterEM3(true);  // Sleep mode
				__enable_irq();
			}

			uint32_t ID_WHO_WOKE_ME_UP = 0x00000000;
			uint8_t AS_RSSI = 0;
			uint8_t AS_OPTIONS = 0;
			// Received Signal Strength Indicator
			if (!AS3933_receive_data(&ID_WHO_WOKE_ME_UP, &AS_RSSI, &AS_OPTIONS, AS3933_SPI_BAUDRATE)) {
				continue;  // Go back to sleep!
			}
		} else {
			ANTENNA_switch(ANTENNA_DEST_CC1101);
			cc1101_spi_init(CC1101_SPI_BAUDRATE);
			cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
			radio_receive_packet(xfer, CC1101_receive, &len, 30);
			while (!(CC1101_receive[0] == 'S' && CC1101_receive[1] == 'T' && CC1101_receive[2] == 'A' && CC1101_receive[3] == 'R' && CC1101_receive[4] == 'T')) {
				memset(CC1101_receive, 0x00, sizeof(CC1101_receive));
				len = 0;
				radio_receive_packet(xfer, CC1101_receive, &len, 30);
			}
		}

		// Awake
		LED_setLED(COL_RED);
		TIMER_delay_ms(TIMER0, 1, false);
		LED_clearLED();

		/* Initial Switch antenna to CC1101 RF Module */
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

		uint8_t CC1101_transmit[16] = { 0 };
		uint8_t idx = 1;
		CC1101_transmit[idx++] = 'A';
		CC1101_transmit[idx++] = 'C';
		CC1101_transmit[idx++] = 'K';

		CC1101_transmit[0] = idx;

		#if USE_CSMA_CA
			radio_send_packet_use_CSMA_CA(xfer, CC1101_transmit, idx);
		#else
			radio_send_packet(xfer, CC1101_transmit, idx);
		#endif
	}
}

void AS_optimization_basestation(struct radio_transfer *xfer) {
	uint8_t paTableData[8] = PA_FSK;

	while(1) {
		uint8_t CC1101_receive[64] = { 0 };
		char str_buffer[128] = { 0 };
		uint8_t len = 0;

		/* Initial Switch antenna to CC1101 RF Module */
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

		radio_receive_packet(xfer, CC1101_receive, &len, 3000);
		while (!(CC1101_receive[0] == 'F' && CC1101_receive[1] == 'I' && CC1101_receive[2] == 'N')) {
			memset(CC1101_receive, 0x00, sizeof(CC1101_receive));
			len = 0;
			radio_receive_packet(xfer, CC1101_receive, &len, 3000);
		}

		uint16_t num_of_packets = (uint16_t) ((CC1101_receive[3] << 8) | CC1101_receive[4]);
		uint16_t num_of_ack = (uint16_t) ((CC1101_receive[5] << 8) | CC1101_receive[6]);
		uint16_t errors = (uint16_t) ((CC1101_receive[7] << 8) | CC1101_receive[8]);

		// Build AVG_RSSI variable
		float avg_rssi = 0.0;
		uint8_t *ptr1 = (uint8_t *) &avg_rssi;
		ptr1[0] = CC1101_receive[9];
		ptr1[1] = CC1101_receive[10];
		ptr1[2] = CC1101_receive[11];
		ptr1[3] = CC1101_receive[12];

		// Build distance variable
		float dist = 0.0;
		uint8_t *ptr2 = (uint8_t *) &dist;
		ptr2[0] = CC1101_receive[13];
		ptr2[1] = CC1101_receive[14];
		ptr2[2] = CC1101_receive[15];
		ptr2[3] = CC1101_receive[16];

		// Build std_dev variable
		float std_dev = 0.0;
		uint8_t *ptr3 = (uint8_t *) &std_dev;
		ptr3[0] = CC1101_receive[17];
		ptr3[1] = CC1101_receive[18];
		ptr3[2] = CC1101_receive[19];
		ptr3[3] = CC1101_receive[20];

		float dist_alternative_computation = 2.74721 / (pow(10, (avg_rssi - 1.9) / 20.0));

		char tmp1[20] = {'\0'};
		char tmp2[20] = {'\0'};
		char tmp3[20] = {'\0'};
		char tmp4[20] = {'\0'};
		ftoa(tmp1, avg_rssi);
		ftoa(tmp2, dist);
		ftoa(tmp3, dist_alternative_computation);
		ftoa(tmp4, std_dev);

		sprintf(str_buffer, "\r\nPackets: %d ACK: %d Errors: %d RSSI: %s STD_DEV: %s Dist_1: %s Dist_2: %s\r\n\r\n", num_of_packets, num_of_ack, errors, tmp1, tmp4, tmp2, tmp3);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));

		LED_setLED(COL_RED);
		RTC_delay_ms(200);
		LED_clearLED();
	}
}

void TEST_WDOG(void) {
	uint16_t i = 0;
	char str_buffer[128] = { 0 };

	if (!WDOG_ENABLED) {
		UART_WriteString("enable WDOG in config.h!", sizeof("enable WDOG in config.h!"));
		while(1);
	}

	WDOG_Feed();		// Reset WDOG

	while(1) {
		RTC_delay_ms(1000);
		i++;
		sprintf(str_buffer, "SECOND %d\r\n", i);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));
	}
}

void testBroadcastTransmitter(struct radio_transfer *xfer, uint8_t config, uint8_t * powerConfig) {

	char str_buffer[64] = { 0 };

	ANTENNA_switch(ANTENNA_DEST_CC1101);
	cc1101_spi_init(CC1101_SPI_BAUDRATE);

	uint8_t broadcastData = 0;

	while(1) {

		sprintf(str_buffer, "BroadcastData: %d\r\n", broadcastData);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));

		cc1101_change_config_to(config, powerConfig);
		cc1101_send_broadcast_packet(xfer, broadcastData++);

		LED_setLED(COL_RED);
		RTC_delay_ms(10);
		LED_clearLED();
		RTC_delay_ms(TEST_WU_DELAY);
	}
}

void testBroadcastReceiver() {
	while(1) {

		if (REVISION == 3) {
			LEUART0_enter_DefaultMode_from_RESET();
			DMA_setup_leuart();  // DMA init
		} else {
			while(1);  // only working in Rev 3
		}

		// Enable AS3933 WakeUp by Address
		#if		WU_ADDR_LENGTH == USE_8_BIT
			AS3933_enable_wakeup_by_8bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
		#elif	WU_ADDR_LENGTH == USE_16_BIT
			AS3933_enable_wakeup_by_16bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
		#elif	WU_ADDR_LENGTH == USE_24_BIT
			AS3933_enable_wakeup_by_24bit_address(TEST_WAKE_UP_ID, AS3933_SPI_BAUDRATE);
		#endif

		// Switch antenna to WakeUp Receiver
		ANTENNA_switch(ANTENNA_DEST_WAKEUPRF);

		BROADCAST_PATTERN = false;
		while (!BROADCAST_PATTERN) {
			__disable_irq();
			EMU_EnterEM2(true);  // Sleep mode
			__enable_irq();
		}

		// Broadcast successful:
		LED_setLED(COL_RED);
		RTC_delay_ms(500);
		LED_clearLED();

		UART_WriteString(DMA_LEUART_RX_BUF, sizeof(DMA_LEUART_RX_BUF));
	}
}

void TEST_data_transmitter(struct radio_transfer *xfer, const uint8_t *key, const uint8_t *iv) {

	uint8_t paTableData[8] = PA_FSK;
	uint8_t paTableWakeUp[8] = PA_TABLE_HIGH_POWER_REDUCED_IF;

	uint32_t MY_ID = 0x2AFFE2;
	uint32_t WAKEUP_ID = 0xCACACA;

	char str_buffer[128] = { 0 };

	while (1) {

		uint8_t CC1101_data[256] = { 0 };
		uint8_t CC1101_send[256] = { 0 };

		/* Initial Switch antenna to CC1101 RF Module */
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_change_config_to(CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);

		#if		WU_ADDR_LENGTH == USE_8_BIT
			cc1101_send_wakeup_packet_8bit_addr(xfer, WAKEUP_ID, MY_ID, AS_OPT_BYTE_DATA_MODE);
		#elif	WU_ADDR_LENGTH == USE_16_BIT
			cc1101_send_wakeup_packet_16bit_addr(xfer, WAKEUP_ID, MY_ID, AS_OPT_BYTE_DATA_MODE, 0);
		#elif	WU_ADDR_LENGTH == USE_24_BIT
			cc1101_send_wakeup_packet_24bit_addr(xfer, WAKEUP_ID, MY_ID, AS_OPT_BYTE_DATA_MODE, 0);
		#endif

		// Build test data packet
		uint8_t i;
		for (i = 0; i < 200; i++) {
			CC1101_data[i] = rand() % 255;
		}

		// Build Header
		RFPacket_build_header(CC1101_send, CC1101_data, &MY_ID, &WAKEUP_ID, CC_OPT_BYTE_DAT_BASIS_TO_IDENTIFIER, &i);

		// Add length byte for CC
		RFPacket_add_cc_length(CC1101_send, &i);

		// Encrypt send packet (AES128CBC)
		RFPacket_encrypt_AES128CBC(CC1101_send, &i, key, iv);

		// Send packet
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
		radio_send_packet(xfer, CC1101_send, i);

		sprintf(str_buffer, "%d Bytes sent (incl. Header)\r\n", i);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));

		LED_setLED(COL_RED);
		RTC_delay_ms(250);
		LED_clearLED();

		RTC_delay_ms(5000);
	}
}

void TEST_data_receiver(struct radio_transfer *xfer, const uint8_t *decrypt_key, const uint8_t *iv) {

	char str_buffer[128] = { 0 };
	uint8_t paTableData[8] = PA_FSK;

	uint32_t MY_ID = 0x1AFFE1;

	uint8_t CC1101_receive[256] = { 0 };
	uint8_t CC_length = 0;

	while (1) {

		// Enable AS3933 WakeUp by Address
		#if		WU_ADDR_LENGTH == USE_8_BIT
			AS3933_enable_wakeup_by_8bit_address(MY_ID, AS3933_SPI_BAUDRATE);
		#elif	WU_ADDR_LENGTH == USE_16_BIT
			AS3933_enable_wakeup_by_16bit_address(MY_ID, AS3933_SPI_BAUDRATE);
		#elif	WU_ADDR_LENGTH == USE_24_BIT
			AS3933_enable_wakeup_by_24bit_address(MY_ID, AS3933_SPI_BAUDRATE);
		#endif

		// Switch antenna to WakeUp Receiver
		ANTENNA_switch(ANTENNA_DEST_WAKEUPRF);

		// Now enter SleepMode!!
		while (!WAKEUP_received) {
			__disable_irq();
			EMU_EnterEM3(true);  // Sleep mode
			__enable_irq();
		}

		uint32_t ID_WHO_WOKE_ME_UP = 0x00;
		uint8_t OPTION_BYTE = 0x00, AS_RSSI = 0x00;
		if (!AS3933_receive_data(&ID_WHO_WOKE_ME_UP, &AS_RSSI, &OPTION_BYTE, AS3933_SPI_BAUDRATE)) {
			continue;  // WakeUp NOT addressed to me ->  Go back to sleep!
		}

		if (OPTION_BYTE != AS_OPT_BYTE_DATA_MODE) {
			continue;
		}

		// Receive
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
		memset(CC1101_receive, 0x00, sizeof(CC1101_receive));
		radio_receive_packet(xfer, CC1101_receive, &CC_length, 3000);

		// Decrypt received packet (AES128CBC)
		RFPacket_decrypt_AES128CBC(CC1101_receive, decrypt_key, iv);

		// Read source address
		uint32_t source_address = (((uint32_t) CC1101_receive[0]) << 16	| ((uint32_t) CC1101_receive[1]) << 8 | CC1101_receive[2]);
		if (source_address != ID_WHO_WOKE_ME_UP) {
			continue;
		}

		// Read destination address
		uint32_t destination_addr = (((uint32_t) CC1101_receive[3]) << 16 | ((uint32_t) CC1101_receive[4]) << 8 | CC1101_receive[5]);
		if (destination_addr != MY_ID) {
			continue;
		}

		// Check Message Type
		uint8_t MESSAGE_TYPE = CC1101_receive[6];
		if (MESSAGE_TYPE != CC_OPT_BYTE_DAT_BASIS_TO_IDENTIFIER) {
			continue;
		}

		// Check CRC
		if (!RFPacket_check_crc16(CC1101_receive)) {
			continue;
		}

		// Set CC1101 RF Module to Data Communication Configuration
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

		LED_setLED(COL_RED);
		RTC_delay_ms(250);
		LED_clearLED();

		sprintf(str_buffer, "%d Bytes received\r\n", CC1101_receive[7]);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));
	}
}

//
void CRC_duration(void) {

	char str_buffer[128] = { 0 };
	RTC_CompareSet(0, (32768 - 1));  // 1 sec

	while(1) {
		// Data buffer
		uint8_t CC1101_data[256] = { 0 };
		double TIMER_VALUE = 0x00;

		// Build test data packet
		uint8_t i;
		for (i = 0; i < 200; i++) {
			CC1101_data[i] = rand() % 255;
		}

		// Add CRC
		//uint16_t crc = crc16(CC1101_data, 0, 200);
		uint16_t crc = CalcBlockCrc(CC1101_data, 200);
		CC1101_data[i++] = crc >> 8;
		CC1101_data[i++] = crc & 0xFF;

		// Check CRC over 200 Bytes Packet
		RTC_Enable(true);
		RTC_CounterReset();
		//uint16_t chk_crc = crc16(CC1101_data, 0, 200);
		uint16_t chk_crc = CalcBlockCrc(CC1101_data, 200);
		if(CC1101_data[200] == (chk_crc >> 8) && CC1101_data[201] == (chk_crc & 0xFF)) {
			TIMER_VALUE = RTC_CounterGet();
			RTC_Enable(false);
		}

		TIMER_VALUE *= 30.517578125;
		char tmp1[20] = {0};
		ftoa(tmp1, TIMER_VALUE);

		sprintf(str_buffer, "%sus\r\n", tmp1);
		UART_WriteString(str_buffer, sizeof(str_buffer));
		memset(str_buffer, '\0', sizeof(str_buffer));

		// Delay
		RTC_delay_ms(5000);
	}
}

void TEST_programmer(struct radio_transfer *xfer, const uint8_t *encrypt_key, const uint8_t *decrypt_key, const uint8_t *iv) {
	char str_buffer[128] = { 0 };

	uint8_t CC1101_receive[256] = { 0 };					// CC1101 Receive Buffer
	uint8_t CC_length = 0;									// CC1101 Length Variable

	uint8_t powerTableBroadcast[] = PA_TABLE_HIGH_POWER_REDUCED_IF;
	uint8_t powerTableData[] = PA_FSK;

	uint32_t GENERAL_BROADCAST = GENERAL_BROADCAST_BASE_ADDR;

	while (1) {
		RTC_delay_ms(3000);

		UART_WriteString("SEARCHING DEVICES:\r\n", sizeof("SEARCHING DEVICES:\r\n"));

		// First send broadcast packet
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		cc1101_change_config_to(CC1101_WAKEUP_CONFIG_NARROW_BAND, powerTableBroadcast);
		cc1101_send_broadcast_packet(xfer, 0x01);

		// Prepare Variables
		uint32_t DEVICE_ADDR = 0x00, DEVICE_WAKEUP_ADDR = 0x00;
		uint64_t UNIQUE_ID = 0x00;
		uint8_t num_of_responses = 0;

		// Listen for responses
		RTC_start(1000);
		while (!RTC_TIMEOUT) {

			// Receive Data from Tables
			memset(CC1101_receive, 0x00, sizeof(CC1101_receive));
			cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, powerTableData);
			radio_receive_packet(xfer, CC1101_receive, &CC_length, 200);

			// Decrypt received packet (AES128CBC)
			RFPacket_decrypt_AES128CBC(CC1101_receive, decrypt_key, iv);

			// Check packet and remove header
			uint8_t MSG_TYPE = CC_OPT_BYTE_MSG_BROADCAST;
			uint32_t SOURCE_ADDR = 0x00;
			uint8_t dat_length = RFPacket_check_and_remove_header(CC1101_receive, &SOURCE_ADDR, &GENERAL_BROADCAST, &MSG_TYPE);

			// Check if Data received
			if (dat_length != 14) {
				continue;
			}

			// Read Device Address
			DEVICE_ADDR = SOURCE_ADDR;

			// Read Device WakeUp Address
			DEVICE_WAKEUP_ADDR = (((uint32_t) CC1101_receive[11]) << 16 | ((uint32_t) CC1101_receive[12]) << 8 | CC1101_receive[13]);

			// Read Device Unique ID
			uint8_t i;
			for (i = 0; i < 8; i++) {
				uint8_t shift = ((7-i)*8);
				UNIQUE_ID |= (uint64_t) CC1101_receive[i+3] << shift;
			}

			// Acknowledge reception of data from table
			cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, powerTableData);
			cc1101_ack_reception(xfer, &GENERAL_BROADCAST, &DEVICE_ADDR, encrypt_key, iv);

			// Increase number of responses
			num_of_responses++;
		}
		TIMER_stop(TIMER0);

		if (num_of_responses != 1) {
			UART_WriteString("ERROR: NOT EXACTLY ONE DEVICE IN RANGE!\r\n", sizeof("ERROR: NOT EXACTLY ONE DEVICE IN RANGE!\r\n"));
			continue;
		} else {
			uint32_t device_id_high = UNIQUE_ID >> 32, device_id_low = UNIQUE_ID;
			sprintf(str_buffer, "DEVICES_IN_RANGE: %d | DEVICE_ID: 0x%X | WAKEUP_ID: 0x%X | UNIQUE_ID: 0x%X%X\r\n", num_of_responses, (int) DEVICE_ADDR, (int) DEVICE_WAKEUP_ADDR, (int) device_id_high, (int) device_id_low);
			UART_WriteString(str_buffer, sizeof(str_buffer));
			memset(str_buffer, '\0', sizeof(str_buffer));
		}

		// Only 1 Device in Range -> Wait for Device to be in listen mode
		RTC_delay_ms(300);

		// DEBUG:
		UART_WriteString("WAKEUP DEVICE ... ", sizeof("WAKEUP DEVICE ... "));

		// WakeUp Device
		cc1101_change_config_to(CC1101_WAKEUP_CONFIG_NARROW_BAND, powerTableBroadcast);
		cc1101_send_wakeup_packet_24bit_addr(xfer, DEVICE_WAKEUP_ADDR, GENERAL_BROADCAST_BASE_ADDR, AS_OPT_BYTE_SERVICE_MODE, 25);

		// Wait for ACK
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, powerTableData);
		bool ack = cc1101_check_ack(xfer, &DEVICE_ADDR, &GENERAL_BROADCAST, decrypt_key, iv, 100);

		if(!ack) {
			UART_WriteString("\r\nERROR: COULDN'T WAKE UP DEVICE!\r\n", sizeof("\r\nERROR: COULDN'T WAKE UP DEVICE!\r\n"));
			continue;
		} else {
			UART_WriteString("OK!\r\n", sizeof("OK!\r\n"));
		}

		// Send Settings
		uint8_t CC1101_send[256] = { 0 };						// CC1101 Send Buffer
		uint8_t CC1101_data[256] = { 0 };						// CC1101 Data Buffer
		CC_length = 0;

		// UNIQUE ID OF MCU
		CC1101_data[CC_length++] = (UNIQUE_ID >> 56);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 48);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 40);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 32);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 24);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 16);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 8);
		CC1101_data[CC_length++] = (UNIQUE_ID >> 0);

		// TODO:
		uint8_t new_encrypt_key[] = AES_ENCRYPTION_KEY;
		uint32_t newID = 0x12ABCD;
		uint32_t newWakeUpAddr = 0x12ABCD;

		// New ID
		CC1101_data[CC_length++] = newID >> 16;
		CC1101_data[CC_length++] = newID >> 8;
		CC1101_data[CC_length++] = newID >> 0;

		// New WAKEUP Address
		CC1101_data[CC_length++] = newWakeUpAddr >> 16;
		CC1101_data[CC_length++] = newWakeUpAddr >> 8;
		CC1101_data[CC_length++] = newWakeUpAddr >> 0;

		// New AES KEY
		CC1101_data[CC_length++] = new_encrypt_key[0];
		CC1101_data[CC_length++] = new_encrypt_key[1];
		CC1101_data[CC_length++] = new_encrypt_key[2];
		CC1101_data[CC_length++] = new_encrypt_key[3];
		CC1101_data[CC_length++] = new_encrypt_key[4];
		CC1101_data[CC_length++] = new_encrypt_key[5];
		CC1101_data[CC_length++] = new_encrypt_key[6];
		CC1101_data[CC_length++] = new_encrypt_key[7];
		CC1101_data[CC_length++] = new_encrypt_key[8];
		CC1101_data[CC_length++] = new_encrypt_key[9];
		CC1101_data[CC_length++] = new_encrypt_key[10];
		CC1101_data[CC_length++] = new_encrypt_key[11];
		CC1101_data[CC_length++] = new_encrypt_key[12];
		CC1101_data[CC_length++] = new_encrypt_key[13];
		CC1101_data[CC_length++] = new_encrypt_key[14];
		CC1101_data[CC_length++] = new_encrypt_key[15];

		// Build Header
		RFPacket_build_header(CC1101_send, CC1101_data, &GENERAL_BROADCAST, &DEVICE_ADDR, CC_OPT_BYTE_MSG_SERVICE, &CC_length);
		// Add length byte for CC1101
		RFPacket_add_cc_length(CC1101_send, &CC_length);
		// Encrypt send packet (AES128CBC)
		RFPacket_encrypt_AES128CBC(CC1101_send, &CC_length, encrypt_key, iv);

		// Debug:
		UART_WriteString("TRANSMITTING NEW SETTINGS TO DEVICE ... ", sizeof("TRANSMITTING NEW SETTINGS TO DEVICE ... "));

		// Send packet
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, powerTableData);
		radio_send_packet(xfer, CC1101_send, CC_length);

		// Check ACK
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, powerTableData);
		ack = cc1101_check_ack(xfer, &DEVICE_ADDR, &GENERAL_BROADCAST, decrypt_key, iv, 300);

		if(!ack) {
			UART_WriteString("\r\nERROR: COULDN'T TRANSMIT DATA TO DEVICE!\r\n", sizeof("\r\nERROR: COULDN'T TRANSMIT DATA TO DEVICE!\r\n"));
			continue;
		} else {
			UART_WriteString("OK!\r\n", sizeof("OK!\r\n"));
		}

		// Check Second ACK
		UART_WriteString("CHECK NEW SETTINGS ... ", sizeof("CHECK NEW SETTINGS ... "));

		uint8_t newDecryptionKey[16];
		AES_DecryptKey128(newDecryptionKey, new_encrypt_key);
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, powerTableData);
		ack = cc1101_check_ack(xfer, &newID, &GENERAL_BROADCAST, newDecryptionKey, iv, 500);

		if(!ack) {
			UART_WriteString("\r\nERROR: COULDN'T OVERWRITE NEW VALUES!\r\n", sizeof("\r\nERROR: COULDN'T OVERWRITE NEW VALUES!\r\n"));
			continue;
		} else {
			UART_WriteString("SUCCESS!\r\n", sizeof("SUCCESS!\r\n"));
		}

		UART_WriteString("FINISHED!\r\n\r\n", sizeof("FINISHED!\r\n\r\n"));
	}
}

void TEST_DWM_UWB_initiator_interrupt()
{
	uint32_t ID_WHOWOKE_ME_UP = 0x0000310D;
	DWM1000_UWB_NODE(ID_WHOWOKE_ME_UP, 25);

	while(1);
}


void TEST_DWM_UWB_responder_interrupt()
{
	float UWB_data[25] = {0};
	uint32_t NODE_ADDRESS = 0x00;

	DWM1000_UWB_TAG(UWB_data, 25, &NODE_ADDRESS);

	while(1);
}


void TEST_DWM_UWB_NODE_SLEEP()
{
	while(1)
	{
		uint8_t wakeup_buffer[50] = {0};
		uint32_t ID_WHOWOKE_ME_UP = 0x0000310D;

		LED_setLED(COL_GREEN);
		RTC_delay_ms(5);
		LED_clearLED();

//DWM1000:
		DWM1000_Chip_INIT();										// INIT the DWM1000 with all the settings
		DWM1000_enter_sleepmode();									// Enter the sleep mode of the DWM1000

// Wake up:
		localization = STATE_WAKE_UP;
		cc1101_power_down();										// Set CC1101 RF Module to WakeUp Configuration and power down
		AS3933_EFM_sleep_enable_wake_up (TEST_WAKE_UP_ID);			// AS3933 enable for wake up, set EFM to sleep

		// Awake
		LED_setLED(COL_RED);
		RTC_delay_ms(5);
		LED_clearLED();

		DWM1000_SPI_Wake_Up(wakeup_buffer, 50);						// wake up DWM1000 with SPI
		DWM1000_UWB_NODE(ID_WHOWOKE_ME_UP, 25);						// start UWB NODE, works as initiator and starts the positioning. 25 positions
	}
}

void TEST_DWM_TAG_SLEEP()
{
	while(1)
	{
		uint8_t wakeup_buffer[50] = {0};
		uint8_t n_measurements = NUMBER_OF_MEASUREMENTS;
		uint32_t NODE_1 = 0;


		float UWB_data[n_measurements];
		memset(UWB_data, 0, n_measurements);

		uint32_t NODE_ADDRESS = 0x00;

		LED_setLED(COL_GREEN);
		RTC_delay_ms(5);
		LED_clearLED();

//DWM1000:
		DWM1000_Chip_INIT();										// INIT the DWM1000 with all the settings
		DWM1000_enter_sleepmode();									// Enter the sleep mode of the DWM1000

// Wake up:
		localization = STATE_WAKE_UP;
		cc1101_power_down();										// Set CC1101 RF Module to WakeUp Configuration and power down
		AS3933_EFM_sleep_enable_wake_up (TEST_WAKE_UP_ID);			// AS3933 enable for wake up, set EFM to sleep

		// Awake
		LED_setLED(COL_RED);
		RTC_delay_ms(5);
		LED_clearLED();

		DWM1000_SPI_Wake_Up(wakeup_buffer, 50);						// wake up DWM1000 with SPI
		DWM1000_UWB_TAG(UWB_data, n_measurements, &NODE_ADDRESS);
		NODE_1 = NODE_ADDRESS;
	}
}

void TEST_DWM_TAG_TO_BASIS()
{
	uint8_t CC1101_full_data[256] = {0};
	uint8_t CC1101_send[256] = {0};
    uint16_t n_measurements = NUMBER_OF_MEASUREMENTS;
    uint8_t wakeup_buffer[50] = {0};
    uint16_t RESPONSE_TIMEOUT = 500;


	// (Prepare variables)
	memset(CC1101_full_data, 0x00, sizeof(CC1101_full_data));
	memset(CC1101_send, 0x00, sizeof(CC1101_send));
	uint8_t known_nodes[256] = {0};
	uint8_t known_nodes_idx = 0, num_of_responses = 0, idx = 1, i = 0, node_high = 0, node_low = 0, iterations = 0, n = 0;
	bool node_already_known = false;

	uint32_t NODE_ADDRESS = 0x00;
	float UWB_data[25];																					// UWB data buffer with n measurements
	memset(UWB_data, 0, n_measurements);

	//uint8_t *UWB_data_pointer;

	//DWM1000_SPI_Wake_Up(wakeup_buffer, 50);																			// wake up DWM1000 with SPI
	DWM1000_Chip_INIT();																							// DWM1000 chip INIT

	for (iterations = 0; iterations < NUM_OF_WAKEUP_ITERATIONS; iterations++)
	{
		RTC_start(RESPONSE_TIMEOUT);																				// Compute max listen Time
		while (!RTC_TIMEOUT)
		{
			DWM1000_UWB_TAG(UWB_data, n_measurements, &NODE_ADDRESS);												// UWB TAG waits for starting NODE with the poll message

			RTC_start(RESPONSE_TIMEOUT);  																			// Reset RTC Timeout after each received node

			node_high = (NODE_ADDRESS >> 8);
			node_low = (NODE_ADDRESS);

			// Check if node has already sent data
			node_already_known = false;
			i = 0;
			while (!node_already_known && i <= known_nodes_idx)
			{
				// Search index of NODE
				if (!(known_nodes[i] == node_high && known_nodes[i+1] == node_low))									// if nodes not known => continue + i++ => back to while
				{
					i++;
					continue;
				}

				// Table found in list
				node_already_known = true;
			}

			// Store the received data
			if (!node_already_known)
			{
				// Add table to known tables list
				known_nodes[known_nodes_idx++] = node_high;
				known_nodes[known_nodes_idx++] = node_low;

				// Increase Number of Responses
				num_of_responses++;

				// Add received Data to send packet
				CC1101_full_data[idx++] = node_high; 					// NODE_ID (high)
				CC1101_full_data[idx++] = node_low; 					// NODE_ID (low)
				CC1101_full_data[idx++] = 0x01;

				uint8_t *UWB_data_pointer = (uint8_t *) UWB_data;		// save float UWB data to uint8 pointer
				for(n = 0; n < (n_measurements * 4); n++)
				{
					CC1101_full_data[idx++] = *UWB_data_pointer;		// get the 4 byte long UWB positions and save it to the CC1101 array
					UWB_data_pointer++;
				}

				//reconvert to float
//				memset(UWB_data, 0, n_measurements);
//
//				float *UWB_float_data_pointer = (float *)CC1101_full_data;
//				for(n = 0; n < n_measurements; n++)
//				{
//					UWB_data[n] = *UWB_float_data_pointer;
//					UWB_float_data_pointer++;
//				}
			}
		}
	}
}


#endif


