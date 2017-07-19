/**************************************************************************//**
 * @file
 * @brief Mobile Tag
 * @author
 * @version
 ******************************************************************************/

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "em_gpio.h"
#include "em_leuart.h"

#include "HWinit.h"
#include "AS3933.h"
#include "ANTENNA.h"
#include "CC1101.h"
#include "RADIO.h"
#include "RADIO_CONFIG.h"
#include "RFPACKETHELPER.h"
#include "EEPROM_handler.h"
#include "WORKMODES.h"
#include "revisionDef.h"
#include "UART.h"
#include "DMA_LEUART.h"
#include "RTC.h"
#include "TIMER.h"
#include "CRC.h"
#include "MISC.h"
#include "TestModes.h"

#include "DWM1000.h"
#include "DWM1000.mem.h"
#include "DWM1000_device_param.h"
#include "DWM1000_reg.h"

#include "enum.h"

/* SYSTEM VARIABLES */
struct radio_transfer xfer;  							// Radio Transfer Handler
#if UART_DEBUG
char UART_data[256] = { 0 }; 							// UART String Buffer
#endif
uint8_t CC1101_receive[256] = { 0 };					// CC1101 Receive Buffer
uint8_t CC1101_send[256] = { 0 };						// CC1101 Send Buffer
//uint8_t CC1101_full_data[1024] = { 0 };					// CC1101 Data full Buffer
uint8_t CC1101_data_node[256] = { 0 };					// CC1101 Data Buffer node
uint8_t CC_length = 0;									// CC1101 Length Variable
uint8_t STATE_OPT_BYTE = 0x00;							// GETSTATE Option Byte
char UART_kbs_data[256] = { 0 }; 						// KBS UART String Buffer
uint16_t UART_kbs_length = 0x00;						// KBS UART data length
bool UART_kbs_busy_flag = false;						// BUSY FLAG for KBS UART DATA

/* GLOBAL VARIABLES */
uint32_t MY_TAG_ID = TAG_ID;
uint32_t MY_WAKEUP_ID = TAG_ID;							// Same as Identifier ID
uint32_t NODE_WAKEUP_ID = GENERAL_NODE_WAKEUP_ID;		// Table WakeUp Address
uint32_t ID_WHO_WOKE_ME_UP = 0x00000000;				// ID of received WakeUp Signal
uint8_t OPTION_BYTE = 0x00;								// Option Byte in AS3933 (Normal mode, Data mode,...)

uint8_t AS_RSSI = 0;				 					// Received Signal Strength Indicator (dB)
int8_t CC_RSSI = 0;  									// Received Signal Strength Indicator (dBm)
uint16_t BAT_mV = 0;									// Battery Status of this node (mV)

/* AES VARIABLES */
uint8_t AES_decryption_key[16];  						// AES Decryption Key
uint8_t AES_encryption_key[] = AES_ENCRYPTION_KEY;		// AES Encryption Key 128 bit
uint8_t AES_decryption_public_key[16];  				// AES Decryption Public Key
uint8_t AES_encryption_public_key[] = AES_ENCRYPTION_PUBLIC_KEY;		// AES Encryption Public Key 128 bit
const uint8_t AES_initVector[] = AES_INIT_VECTOR;  		// Initialization vector used during CBC

/* UWB VARIABLES */

uint8_t BLINK_message[]= BLINK_MESSAGE;					// Blink message
uint8_t POLL_message[] = POLL_MESSAGE;					// Poll message
uint8_t RESP_message[] = RESPONSE_MESSAGE;				// Response message
uint8_t FINAL_message[] = FINAL_MESSAGE;   				// Final message
uint8_t wakeup_buffer[50] = {0};						// wake up buffer to wake up DWM1000 Module
uint16_t N_MEASUREMENTS = NUMBER_OF_MEASUREMENTS;		// number of measurements with UWB
uint16_t NODE_IDs[30] = { 0 };
uint8_t NODE_NUMBERS = 0;

enum states_tag state = STATE_SEND_BLINK;
enum states_node state1 = STATE_POLL_TRANSMIT;
enum states_localization localization = STATE_WAKE_UP;

/* PA table settings, see config.h */
uint8_t paTableWakeUp[8] 	= PA_TABLE_HIGH_POWER_REDUCED_IF;
uint8_t paTableDataProgrammer[8]   = PA_FSK_ULTRA_LOW;
uint8_t paBaseData[8] 		= PA_FSK;					// 0dBm
#if REDUCED_FSK_POWER
uint8_t paTableData[8]   	= PA_FSK_LOW;  				// 0dBm
#else
uint8_t paTableData[8]   	= PA_FSK;					// 12dBm
#endif

/* CSMA SETTINGS */
const uint16_t SLOT_SIZE = 25;							// Size of s single CSMA Time Slot in ms
const uint16_t MAX_NUM_OF_SENDS = 5;					// Maximum Number of repeating in case of failed transmissions
const uint16_t MAX_NUM_OF_SLOTS = 50;					// Maximum number of possible time slots
uint8_t MAX_NUM_OF_TRIES = 10;							// Maximum number of (successful but not acknowledged) tries to send data

/* TIMESLOT SETTINGS */
#if USE_CSMA_CA
	uint16_t RESPONSE_TIMEOUT = 75;						// Timeout when no more responses from tables
#else
	#define NUM_OF_TIMESLOTS  		10					// Max. number of responding table
	#define SEND_DURATION  			50					// Time needed to send out packet
	#define ACKTIME  				25					// Time needed for ACK from Identifier
	#define TABLE_MAX_NUM_OF_TRIES  3					// Maximum number of tries to send data from table to identifier
	#define OFFSET  				10					// Offset if time slot == 0
	uint16_t RESPONSE_TIMEOUT = ((NUM_OF_TIMESLOTS + 1) * ((SEND_DURATION + ACKTIME) * TABLE_MAX_NUM_OF_TRIES) + OFFSET);	// Size of time slot
#endif

/* INTERRUPT ROUTINES DECLARATION */
void TIMER1_IRQHandler(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

#if (REVISION == IDENTIFIER_REV_3) && BROADCASTS_ENABLED
void LEUART0_IRQHandler(void);
#endif

int main(void){
	/* Chip errata */

	CHIP_Init();

	/* Initialize HW (SPI, UART, GPIO, CLOCKS, ...) */
	enter_DefaultMode_from_RESET();

	/* Initialize RTC for Timing Purposes */
	RTC_init(1000);

	/* Initialize Timer 0 for milliseconds */
	TIMER_init(TIMER0, cmuClock_TIMER0, TIMER0_IRQn);

	// Initialize Timer 2 for microseconds
	TIMER_init_us(TIMER2, cmuClock_TIMER2, TIMER2_IRQn);

	/* Switch off LEDs */
	LED_clearLED();

	/* Initial Switch antenna to CC1101 RF Module */
	ANTENNA_switch(ANTENNA_DEST_CC1101);

	/* Set SPI of DWM1000 to default CS High mode  */
	DWM1000_spi_default();

	/*Initialize UWB Module and enter sleep mode*/
	localization = STATE_UWB;
	DWM1000_Chip_INIT();									// INIT the DWM1000 with all the settings
	DWM1000_enter_sleepmode();								// Enter the sleep mode of the DWM1000

	/* Initialize CC1101 RF Module */
	localization = STATE_WAKE_UP;
	cc1101_spi_init(CC1101_SPI_BAUDRATE);
	cc1101_change_config_to(CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	radio_init(&xfer, TIMER1, cmuClock_TIMER1, TIMER1_IRQn);
	/* Initialize Watchdog */
	#if WDOG_ENABLED && (MODE == NORMAL || MODE == TEST_WATCHDOG_FUNCTION)
		if (RMU_ResetCauseGet() & RMU_RSTCAUSE_WDOGRST)
		{
			// Something went wrong -> WDOG triggered
			#if UART_DEBUG
				UART_WriteString("ERROR: WDOG triggered!!\r\n", sizeof("ERROR: WDOG triggered!!\r\n"));
			#endif
		}

		// (Re-)initialize WDOG
		RMU_ResetCauseClear();
		WDOG_enter_DefaultMode_from_RESET();
		WDOG_Feed();
	#endif

	/* Initialize EEPROM */
	#if USE_EEPROM_VALUES
		EEPROM_init();
		if (EEPROM_OVERWRITE_VALUES_ON_START)
		{
			EEPROM_write(&MY_TAG_ID, &MY_WAKEUP_ID, AES_encryption_key);
		}
		EEPROM_read_and_store_into(&MY_TAG_ID, &MY_WAKEUP_ID, AES_encryption_key);
	#endif

	/* Calculate (once) decryption key from the original key */
	AES_DecryptKey128(AES_decryption_key, AES_encryption_key);
	AES_DecryptKey128(AES_decryption_public_key, AES_encryption_public_key);

	AS3933_prepare_wakeup(AS3933_SPI_BAUDRATE, true);
	AS3933_wakeup_pattern_16bit((uint16_t)MY_WAKEUP_ID);
	AS3933_start_wakeup();

	/* Optional test modes, to set in config.h */
	#if MODE != NORMAL
	TESTMODE_enter(&xfer);
	#endif

	while(1)
	{
		// Blink LED
		//LED_setLED(COL_GREEN);
		//RTC_delay_ms(5);
		//LED_clearLED();

		// Feed Watchdog
		#if WDOG_ENABLED
		WDOG_Feed();
		#endif

/*
 * Go into Sleep Mode (EFM) and enable AS3933 WakeUp
 */
		cc1101_power_down();										// Set CC1101 RF Module to WakeUp Configuration and power down
		AS3933_EFM_sleep_enable_wake_up(MY_WAKEUP_ID);				// AS3933 enable for wake up, set EFM to sleep

/**************************************************************************************************************
 * Normal WakeUp -> Read AS3933 data
 **************************************************************************************************************/
		ID_WHO_WOKE_ME_UP = 0x00;
		OPTION_BYTE = 0x00;
		if (!AS3933_receive_data(&ID_WHO_WOKE_ME_UP, &OPTION_BYTE, &N_MEASUREMENTS, AS3933_SPI_BAUDRATE))
		{
		// WakeUp NOT addressed to me (Additional addr byte -> no match)
			#if UART_DEBUG
				UART_WriteString("WakeUp NOT addressed to me\r\n", sizeof("WakeUp NOT addressed to me\r\n"));
			#endif
				continue;  // Go back to sleep!
		}

		// Disable LEUART for broadcast listening
		#if REVISION == IDENTIFIER_REV_3 && BROADCASTS_ENABLED
			LEUART_Enable(LEUART0, leuartDisable);
		#endif

/*************************************************************************************************************
 * 										ACK to Base Station!																					   ***
 *************************************************************************************************************/
		// Switch antenna to CC1101 RF module and reconfigure SPI
		ANTENNA_switch(ANTENNA_DEST_CC1101);
		cc1101_spi_init(CC1101_SPI_BAUDRATE);

/*
 * BASE CMD: find
 */
		// ACK WakeUp
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);
		cc1101_ack_reception(&xfer, &MY_TAG_ID, &ID_WHO_WOKE_ME_UP, AES_encryption_key, AES_initVector);

		// check number of measurements

		if(N_MEASUREMENTS == 0)											   // If number of measurements is zero, choose the default value in NUMBER_OF_MEASUREMENTS
		{
			N_MEASUREMENTS = NUMBER_OF_MEASUREMENTS;
		}

		// Receive Node IDs

		RTC_start(1200);													// 2000

		while (!RTC_TIMEOUT)
		{
			int i = 0;
			int j = 1;
			uint8_t MSG_TYPE = CC_OPT_BYTE_DAT_BASIS_TO_IDENTIFIER;
			memset(CC1101_receive, 0x00, sizeof(CC1101_receive));
			// Receive Data from Identifier
			radio_receive_packet(&xfer, CC1101_receive, &CC_length, 100);

			// Decrypt received packet (AES128CBC)
			RFPacket_decrypt_AES128CBC(CC1101_receive, AES_decryption_key, AES_initVector);

			if (!RFPacket_check_and_remove_header(CC1101_receive, &ID_WHO_WOKE_ME_UP, &MY_TAG_ID, &MSG_TYPE))
			{
				// Packet is not addressed to me or has the wrong type
				continue;
			}

			NODE_NUMBERS = CC1101_receive[0];
			for(i = 0; i < NODE_NUMBERS; i++)
			{
				NODE_IDs[i] = (((uint16_t) CC1101_receive[j]) << 8 | ((uint16_t) CC1101_receive[j+1]));
				j++;
				j++;
			}
			 //Set CC1101 RF Module to Data Communication Configuration
			cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);
			// Acknowledge reception of data from BASIS
			cc1101_ack_reception(&xfer, &MY_TAG_ID, &ID_WHO_WOKE_ME_UP, AES_encryption_key, AES_initVector);
		}

		cc1101_power_down();										// Set CC1101 RF Module to WakeUp Configuration and power down

		//UART_WriteString("Got NODE IDs\r\n", sizeof("Got NODE IDs\r\n"));
/**************************************************************************************************************
 * 									RANGING																	***
 *************************************************************************************************************/

		// (Prepare variables)
		uint8_t CC1101_full_data[650] = { 0 };														// CC1101 Data full Buffer
		memset(CC1101_full_data, 0x00, sizeof(CC1101_full_data));
		memset(CC1101_send, 0x00, sizeof(CC1101_send));
		uint8_t response = 1, i = 0, node_high = 0, node_low = 0;
		uint16_t idx = 0, n = 0;

		float UWB_data[500];																		// UWB data buffer with n measurements
		memset(UWB_data, 0, 500);

		DWM1000_SPI_Wake_Up(wakeup_buffer, 50);														// wake up DWM1000 with SPI																										// DWM1000 chip INIT
		DWM1000_Chip_INIT();

		while(response)
		{
			for (i = 0; i < NODE_NUMBERS; i++)														// routine depend on how many nodes
			{
				char UART_data[256] = {0};
				DWM1000_UWB_TAG(UWB_data, N_MEASUREMENTS, NODE_IDs[i]);								// UWB TAG starts with Blink for each NODE

				//sprintf(UART_data,"Received Packet from NODE: 0x%X \r\n",(int) NODE_IDs[i]);
				//UART_WriteString(UART_data, strlen(UART_data));

				node_high = (NODE_IDs[i] >> 8);
				node_low = (NODE_IDs[i]);

				// Add received Data to send packet
				CC1101_full_data[idx++] = node_high; 												// NODE_ID (high)
				CC1101_full_data[idx++] = node_low; 												// NODE_ID (low)

				uint8_t *UWB_data_pointer = (uint8_t *) UWB_data;									// save float UWB data to uint8 pointer
				for(n = 0; n < (N_MEASUREMENTS * 4); n++)
				{
					CC1101_full_data[idx++] = *UWB_data_pointer;									// get the 4 byte long UWB positions and save it to the CC1101 array
					UWB_data_pointer++;
				}
				memset(UWB_data, 0, 500);
			}
			response = 0;
		}

		DWM1000_enter_sleepmode();																	// Enter the sleep mode of the DWM1000

		localization = STATE_WAKE_UP;

		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);

		uint8_t CC1101_part_data[256] = { 0 };
		uint16_t total_packets = 0;
		uint8_t send_length;
		uint16_t act_packet;

		total_packets = ((idx / 226) + 1);

		for(act_packet = 1; act_packet <= total_packets ; act_packet++)
		{
			if (idx > act_packet * 226)														// is the length greater than i times the packet length
			{
				send_length = 226;
			}
			else
			{
				send_length = idx - ((act_packet-1) * 226);
			}
			cc1101_arraycpy(CC1101_part_data, CC1101_full_data, (act_packet-1) * 226, 226);

			RFPacket_build_header_DWM1000(CC1101_send, CC1101_part_data, &MY_TAG_ID, &ID_WHO_WOKE_ME_UP, CC_OPT_BYTE_MSG_IDENTIFIER_TO_BASIS,
					&send_length, act_packet, total_packets);
			RFPacket_add_cc_length_DWM1000(CC1101_send, &send_length);
			RFPacket_encrypt_AES128CBC(CC1101_send, &send_length, AES_encryption_key, AES_initVector);

			/*
			 * Send Data to Basis Station
			 */

			// Send Data to Basis Station
			#if USE_CSMA_CA
				RTC_start(50);
				cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);
				if (radio_send_packet_use_CSMA_CA(&xfer, CC1101_send, send_length))
				{
					// Check if Base ACK my packet
					cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);
					bool acknowledged = cc1101_check_ack(&xfer, &ID_WHO_WOKE_ME_UP, &MY_TAG_ID, AES_decryption_key, AES_initVector, 75);

					if (acknowledged)	// ACK!
					{
						#if UART_DEBUG
						UART_WriteString("BASE-STATION ACK!\r\n\r\n", sizeof("BASE-STATION ACK!\r\n\r\n"));
						#endif
						//break; // ACK! -> leave while
					}
				}
				while(!RTC_TIMEOUT);
			#else
				// Send out packet
				cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);
				radio_send_packet(xfer, CC1101_send, send_length);
				// Check if Base ACK my packet
				cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paBaseData);
				bool acknowledged = cc1101_check_ack(xfer, &ID_WHO_WOKE_ME_UP, &MY_IDENTIFIER_ID, AES_decryption_key, AES_initVector, 75);

				if (acknowledged) {  // ACK!
					#if UART_DEBUG
					UART_WriteString("BASE-STATION ACK!\r\n\r\n", sizeof("BASE-STATION ACK!\r\n\r\n"));
					#endif
					break; // ACK! -> leave while
				}

				#if UART_DEBUG
				UART_WriteString("BASE-STATION NOT ACK!\r\n", sizeof("BASE-STATION NOT ACK!\r\n"));
				#endif
			#endif
		}
	}
}

// Radio timeout timer
void TIMER1_IRQHandler(void) {
	__disable_irq();
	radio_handle_timeout(&xfer);
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	__enable_irq();
}

// CC1101 GDO0 Interrupt / AS3933 WakeUp Interrupt
void GPIO_EVEN_IRQHandler(void)
{
	if(localization == STATE_WAKE_UP)
	{
		if (GPIO_IntGetEnabled() & (1 << CC1101_GDO0_PIN))
		{
			radio_handle_transfer_end(&xfer);
			/* Clear interrupt flag. */
			GPIO_IntClear(1 << CC1101_GDO0_PIN);
		}
		#if 	REVISION == IDENTIFIER_REV_1 || REVISION == IDENTIFIER_REV_2
			if (WAKEUP_active)
			{
				if (GPIO_IntGetEnabled() & (1 << AS3933_WAKE_PIN))
				{
					GPIO_IntDisable(1 << AS3933_WAKE_PIN);
					GPIO_IntClear(1 << AS3933_WAKE_PIN);
					WAKEUP_received = true;
				}
			}
		#endif
	}

// DWM1000 IRQ

	else
	{
		if (GPIO_IntGetEnabled() & (1 << DWM1000_IRQ_PIN))
		{
			DWM1000local.sysstatus = 0;
			DWM1000local.sysstatus = (DWM1000_read32bitoffsetregister(SYS_STATUS_ID,0));

		#if		MODULE == NODE

			if (state1 == STATE_WAIT_RESPONSE_RECEIVE)
			{
				if(DWM1000local.sysstatus & SYS_STATUS_RXFCG)
				{
					state1 = STATE_RESPONSE_RECEIVE;
				}
				else
				{
					DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR);
					state1 = STATE_POLL_TRANSMIT;
				}
			}
		#endif

		#if		MODULE == TAG

			if (state == STATE_RECEIVE_MODE)
			{
				if(DWM1000local.sysstatus & SYS_STATUS_RXFCG)
				{
					state = STATE_POLL_RECEIVE;
				}
				else
				{
					DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR);
					state = STATE_RECEIVER_ON;
				}
			}
			if (state == STATE_WAIT_FINAL_RECEIVE)
			{
				if(DWM1000local.sysstatus & SYS_STATUS_RXFCG)
				{
					state = STATE_FINAL_RECEIVE;
				}
				else
				{
					DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR);
					state = STATE_RECEIVER_ON;
				}
			}
		#endif

			GPIO_IntClear(1 << DWM1000_IRQ_PIN);
		}
	}
}

// CC1101 GDO2 Interrupt / AS3933 WakeUp Interrupt
void GPIO_ODD_IRQHandler(void)
{
	if (GPIO_IntGetEnabled() & (1 << CC1101_GDO2_PIN))
	{
		radio_handle_transfer_threshold(&xfer);
		/* Clear interrupt flag. */
		GPIO_IntClear(1 << CC1101_GDO2_PIN);
	}

	if (WAKEUP_active)
	{
		if (GPIO_IntGetEnabled() & (1 << AS3933_WAKE_PIN))
		{
			GPIO_IntDisable(1 << AS3933_WAKE_PIN);
			GPIO_IntClear(1 << AS3933_WAKE_PIN);
			WAKEUP_received = true;
		}
	}
}
