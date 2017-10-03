/**************************************************************************//**
 * @file
 * @brief NODE
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
uint8_t CC1101_full_data[256] = { 0 };					// CC1101 Data full Buffer
uint8_t CC1101_data_node[256] = { 0 };					// CC1101 Data Buffer node
uint8_t CC_length = 0;									// CC1101 Length Variable

/* GLOBAL VARIABLES */
uint32_t MY_NODE_ID = NODE_ID;							// NODE ID
uint32_t MY_WAKEUP_ID = NODE_WAKEUP_ID;					// NODE WakeUp Address
uint32_t ID_WHO_WOKE_ME_UP = 0x00000000;				// ID of received WakeUp Signal
uint8_t OPTION_BYTE = 0x00;								// Option Byte in AS3933 (Normal mode, Data mode,...)
uint8_t AS_RSSI = 0;				 					// Received Signal Strength Indicator (dB)
int8_t CC_RSSI = 0;  									// Received Signal Strength Indicator (dBm)
uint16_t BAT_mV = 0;									// Battery Status of this node (mV)

/* AES VARIABLES */
uint8_t AES_decryption_key[16];  						// AES Decryption Key
uint8_t AES_encryption_key[] = AES_ENCRYPTION_KEY; 		// AES Encryption Key 128 bit
const uint8_t AES_initVector[] = AES_INIT_VECTOR;  		// Initialization vector used during CBC
uint8_t AES_decryption_public_key[16];  				// AES Decryption Key
uint8_t AES_encryption_public_key[] = AES_ENCRYPTION_PUBLIC_KEY;	// AES Encryption Key 128 bit

/* UWB VARIABLES */
uint8_t BLINK_message[]= BLINK_MESSAGE;					// Blink message
uint8_t POLL_message[] = POLL_MESSAGE;					// Poll message
uint8_t RESP_message[] = RESPONSE_MESSAGE;				// Response message
uint8_t FINAL_message[] = FINAL_MESSAGE;   				// Final message
uint8_t wakeup_buffer[50] = {0};						// wake up buffer to wake up DWM1000 Module
uint16_t N_MEASUREMENTS;

enum states_tag state = STATE_SEND_BLINK;
enum states_node state1 = RECEIVER_ON_STATE;
enum states_localization localization = STATE_WAKE_UP;

/* PA table settings, see config.h */
uint8_t paTableWakeUp[8] 	= PA_TABLE_HIGH_POWER_REDUCED_IF;
uint8_t paTableDataProgrammer[8] = PA_FSK_ULTRA_LOW;
uint8_t paTableBroadcast[8] = PA_FSK;					// 0dBm
#if REDUCED_FSK_POWER
uint8_t paTableData[8]   	= PA_FSK_LOW;  				// 0dBm
#else
uint8_t paTableData[8]   	= PA_FSK;					// 12dBm
#endif

/* TIMESLOT SETTINGS */
#if USE_CSMA_CA
	const uint16_t SLOT_SIZE = 25;			// Size of Time Slot in ms
	const uint16_t MAX_NUM_OF_SENDS = 5;	// Maximum Number of repeating in case of failed transmissions to Identifier
	const uint16_t MAX_NUM_OF_SLOTS = 50;	// Maximum number of possible time slots
#else
	const uint16_t NUM_OF_TIMESLOTS = 10;	// Max. number of responding table
	const uint16_t SEND_DURATION = 50;		// Time needed to send out packet
	const uint16_t ACKTIME = 25;			// Time needed for ACK from Identifier
	const uint16_t MAX_NUM_OF_TRIES = 3;	// Maximum number of tries to send data to Identifier
	const uint16_t OFFSET = 10;				// Offset for first time slot
#endif

/* INTERRUPT ROUTINES DECLARATION */
void TIMER1_IRQHandler(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

#if (REVISION == IDENTIFIER_REV_3 || REVISION == TABLE_REV_1) && BROADCASTS_ENABLED
void LEUART0_IRQHandler(void);
#endif

uint8_t counterDebug = 0; // Yitong
void blink_LED(uint8_t ms, uint8_t loopNum)// Yitong
{
	for (int iyyi=0; iyyi<loopNum; iyyi++ )
	{
		LED_setLED(COL_RED);
		RTC_delay_ms(ms);
		LED_clearLED();
		RTC_delay_ms(ms);
	}
}

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
	DWM1000_Chip_INIT();										// INIT the DWM1000 with all the settings
	DWM1000_enter_sleepmode();									// Enter the sleep mode of the DWM1000

	/* Initialize CC1101 RF Module */
	//cc1101 is not used in node, can be disabled it. Yitong
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
			EEPROM_write(&MY_NODE_ID, &MY_WAKEUP_ID, AES_encryption_key);
		}
		EEPROM_read_and_store_into(&MY_NODE_ID, &MY_WAKEUP_ID, AES_encryption_key);
	#endif

	/* Calculate (once) decryption key from the original key */
	AES_DecryptKey128(AES_decryption_key, AES_encryption_key);
	AES_DecryptKey128(AES_decryption_public_key, AES_encryption_public_key);

	/* Optional test modes, to set in config.h */
	#if MODE != NORMAL
	TESTMODE_enter(&xfer);
	#endif

	AS3933_prepare_wakeup(AS3933_SPI_BAUDRATE, true);
	AS3933_wakeup_pattern_16bit(MY_WAKEUP_ID);
	AS3933_start_wakeup(); // try to disable it. Yitong
	RTC_delay_ms(5); // add by Yitong

	while(1)
	{
		// Blink LED
		LED_setLED(COL_GREEN);
		RTC_delay_ms(10);
		LED_clearLED();

//		LED_setLED(COL_RED);
//		RTC_delay_ms(10);
//		LED_clearLED();

		//blink_LED(10,5);

		// Feed Watchdog
		#if WDOG_ENABLED
		WDOG_Feed();
		#endif

		/*
		 * Go into Sleep Mode (EFM) and enable AS3933 WakeUp
		 */
		localization = STATE_WAKE_UP;
		//Try to disable , Yitong
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
				/*
				for (counterDebug=1; counterDebug<10; counterDebug=counterDebug+1) {
					LED_setLED(COL_RED);
					RTC_delay_ms(50);
					LED_clearLED();
					RTC_delay_ms(50);
				}
				*/
				continue;  // Go back to sleep
		}

		// Disable LEUART for broadcast listening
		#if REVISION == IDENTIFIER_REV_3 && BROADCASTS_ENABLED
			LEUART_Enable(LEUART0, leuartDisable);
		#endif

		//localization = STATE_UWB;  //add by yitong
		DWM1000_SPI_Wake_Up(wakeup_buffer, 50);						// Wake up DWM1000 Module																						// wake up DWM1000 with SPI
		DWM1000_Chip_INIT();										// init the DWM1000 Module

		//RTC_delay_ms(5); // add by Yitong
		DWM1000_UWB_NODE(N_MEASUREMENTS);							// start UWB NODE, works as initiator and starts the positioning. 25 positions

		RTC_delay_ms(5); // add by Yitong
		DWM1000_enter_sleepmode();									// Enter the sleep mode of the DWM1000
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
		if (GPIO_IntGetEnabled() & (1 << CC1101_GDO0_PIN)) //pin 6 in cc1101, pin 23 in EFM32
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

			if (state1 == STATE_WAIT_BLINK_RECEIVE)
			{
				if(DWM1000local.sysstatus & SYS_STATUS_RXFCG)
				{
					state1 = STATE_BLINK_RECEIVE;
				}
				else
				{
					DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, SYS_STATUS_ALL_RX_ERR);
					state1 = RECEIVER_ON_STATE;
				}
			}

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
