#ifndef CONFIG_H_
#define CONFIG_H_

#include "WORKMODES.h"
#include "revisionDef.h"
#include "DWM1000_device_param.h"


static volatile dwt_config_t config = {
	1,               						/* Channel number. */
	DWT_PRF_16M,     						/* Pulse repetition frequency. */
	DWT_PLEN_2048,    						/* Preamble length. */
	DWT_PAC64,        						/* Preamble acquisition chunk size. Used in RX only. */
	4,               						/* TX preamble code. Used in TX only. 16 MHz PRF: 4, 64 MHz PRF: 10 */
	4,               						/* RX preamble code. Used in RX only. */
	1,               						/* Use non-standard SFD (Boolean) */
	DWT_BR_110K,      						/* Data rate. */
	DWT_PHRMODE_STD, 						/* PHY header mode. */
	129,    								/* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.(129 + 8 - 8) */
	0										/* Smart TX Power */
};

/* Byte 0-1: 	Frame control
 * Byte 2: 		sequence number
 * Byte 3-4:	PAN ID
 * Byte 5-6:	Destination ID
 * Byte 7-8:	Source ID
 * Byte 9:		Option/Function code
 *
 * RESPONSE:
 * Byte 10: 	activity code (0x02: go on with ranging)
 * Byte 11-12: 	activity parameter
 *
 * FINAL:
 *
 * Byte 10-13: 	poll message transmission timestamp (TX)
 * Byte 14-17:  response message reception timestamp
 * Byte 18-21: 	final message transmission timestamp
 */

#define BLINK_MESSAGE						{0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xC5, 0, 0}
#define POLL_MESSAGE  						{0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0x21, 0, 0}
#define RESPONSE_MESSAGE  					{0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0x10, 0x02, 0, 0, 0, 0}
#define FINAL_MESSAGE						{0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#define BLINK_MESSAGE_LENGTH				12
#define POLL_MESSAGE_LENGTH					12
#define RESPONSE_MESSAGE_LENGTH				15
#define FINAL_MESSAGE_LENGTH				27

#define DESTINATION_ID_POSITION				5
#define SOURCE_ID_POSITION					7
#define POLL_TX_TIMESTAMP_POSITION			10
#define RESP_RX_TIMESTAMP_POSITION			15
#define FINAL_TX_TIMESTAMP_POSITION			20
#define TIMESTAMP_LENGTH					5


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 4.43 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4430
/* Delay between frames, in UWB microseconds. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 3.75 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 3750//3642
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547
/* Antenna delay values */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436


//=========================================================
// Global configuration file for identifier device
//=========================================================

/* NODE DEFINES */
#define NODE_WAKEUP_ID		  				0xCCCC
#define NODE_ID								0x3E3E
#define TAG_ID 								0x310D		//0x00AC310D

/* AES SETTINGS */
#define AES_ENCRYPTION_KEY					{ 0x4D, 0x35, 0x32, 0x45, 0x4C, 0x39, 0x41, 0x42, 0x48, 0x32, 0x32, 0x53, 0x4C, 0x4A, 0x33, 0x4A }
#define AES_ENCRYPTION_PUBLIC_KEY			{ 0x44, 0x79, 0x62, 0x51, 0x63, 0x5B, 0x78, 0x4D, 0x2E, 0x2C, 0x51, 0x2E, 0x28, 0x56, 0x4B, 0x5A }
#define AES_INIT_VECTOR						{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F }


// PCB REVISION
#define REVISION							TABLE_REV_1
// NODE or TAG
#define MODULE								NODE

//#define MODE								TEST_DWM1000_INITIATOR_IRQ

#define MODE								NORMAL

// Choose WakeUp Address Length (options below):
#define WU_ADDR_LENGTH						USE_16_BIT
// Use CSMA when sending out data
#define USE_CSMA_CA							true
// Enable Broadcast WakeUps
#define BROADCASTS_ENABLED					false
// Enable KBS UART FUNCTION
#define KBS_UART_ENABLED					false
// Enable Busy Flag (DLE) for KBS UART
#define KBS_BUSY_FLAG_1_ENABLED				false
#define KBS_BUSY_FLAG_2_ENABLED				false
// Store Values in EEPROM
#define USE_EEPROM_VALUES					true
// Initial write values in EEPROM
#define EEPROM_OVERWRITE_VALUES_ON_START	true
// Enable WatchDog
#define WDOG_ENABLED						true
// WatchDog Timeout in seconds (roughly)
#define WDOG_TIMEOUT						WDOG_TIMEOUT_8
// Number of sending wakeup packets to tables
#define NUM_OF_WAKEUP_ITERATIONS			2
// HF frequency in Hz: 869MHz, 915MHz
#define HF_FREQ								869
// LF frequency in Hz: 19200Hz, 125000Hz
#define LF_FREQ								19200
// RF Communication with Tables with reduced power
#define REDUCED_FSK_POWER					true


/* OPTIONS: Device modes */
#define NORMAL								1
#define TEST_WAKE_UP_TRANSMITTER			2
#define TEST_WAKE_UP_RECEIVER				3
#define TEST_WAKE_UP_SENSITIVITY			4
#define TEST_SPECTRAL_ANALYSIS				5
#define TEST_SPECTRAL_ANALYSIS_HF_CARRIER	6
#define TEST_AS_OPTIMIZATION_TRANSMITTER	7
#define TEST_AS_OPTIMIZATION_RECEIVER		8
#define TEST_AS_OPTIMIZATION_BASE			9
#define TEST_WATCHDOG_FUNCTION				10
#define TEST_BROADCAST_TRANSMITTER			11
#define TEST_BROADCAST_RECEIVER				12
#define TEST_DATA_TRANSMITTER				13
#define TEST_DATA_RECEIVER					14
#define TEST_PROGRAMMER						15
#define TEST_DWM1000_INITIATOR_IRQ			16
#define TEST_DWM1000_RESPONDER_IRQ			17
#define TEST_DWM1000_NODE_SLEEP				18
#define TEST_DWM1000_NODE_TAG				19


/* OPTIONS: WakeUp Address Length */
#define USE_8_BIT							1
#define USE_16_BIT							2
#define USE_24_BIT							3

/* OPTIONS: WDOG Timeout */
#define WDOG_TIMEOUT_2						wdogPeriod_2k
#define WDOG_TIMEOUT_4						wdogPeriod_4k
#define WDOG_TIMEOUT_8						wdogPeriod_8k
#define WDOG_TIMEOUT_16						wdogPeriod_16k
#define WDOG_TIMEOUT_32						wdogPeriod_32k
#define WDOG_TIMEOUT_64						wdogPeriod_64k
#define WDOG_TIMEOUT_128					wdogPeriod_128k
#define WDOG_TIMEOUT_256					wdogPeriod_256k

// SETTINGS: WAKE_UP_TEST mode config
#define TEST_WAKE_UP_ID						0xCCCC						//0xCACACA
#define TEST_WAKE_UP_OPTION					AS_OPT_BYTE_NORMAL_MODE
#define TEST_WU_DELAY						3000 // in ms

/* SYSTEM SETTINGS */
#define CC1101_SPI_BAUDRATE					5000000ULL  // (5.0 MHz) // max. 6500000ULL  (max 6.5 MHz)
#define AS3933_SPI_BAUDRATE					5000000ULL  // (5.0 MHz) // max. 6000000ULL  (max 6.0 MHz)
#define DWM1000_SPI_BAUDRATE_LOW			2000000ULL  // (2.0 Mhz)
#define DWM1000_SPI_BAUDRATE_HIGH			16000000ULL // (16.0 MHz)

// USE UART DEBUG MESSAGES
#if !KBS_UART_ENABLED
#define UART_DEBUG							false
#endif
#define USE_ADC_IF_NO_UART_BAT_RECEIVED 	false

// OPTIONS: OOK shaping
#define PA_TABLE_LOW_POWER					{0x00,0x32,0x34,0x36,0x38,0x3A,0x3C,0x3F} // recommended for additional power amplifier
#define PA_TABLE_HIGH_POWER_BROADBAND		{0x00,0x03,0x0F,0xCF,0xCE,0xC9,0xC4,0xC0}
#define PA_TABLE_OOK_BROADBAND				{0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xC0} // not tested so far
#define PA_TABLE_HIGH_POWER_REDUCED_IF		{0xCF,0xCF,0xCE,0xCB,0xC7,0xC4,0xC1,0xC0} // recommended for regular wake-up
#define PA_TABLE_CARRIER					{0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0}

// OPTIONS: FSK
#define PA_FSK								{0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00}	// +12dBm
#define PA_FSK_LOW							{0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00}	// +0dBm
#define PA_FSK_ULTRA_LOW					{0x1D,0x00,0x00,0x00,0x00,0x00,0x00,0x00}	// -15dBm


#endif /* CONFIG_H_ */
