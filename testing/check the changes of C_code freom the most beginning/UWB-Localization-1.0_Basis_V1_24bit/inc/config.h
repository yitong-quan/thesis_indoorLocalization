//=========================================================
// Global configuration file for device
//=========================================================

/* PCB revision
 * is equivalent to hardware revision, i.e. 1, 2
 * */
#include "revisionDef.h"

// Print detailed debug information
#define PRINT 						false

#define REVISION					BASE_REV_2

// Choose mode (options below):
#define MODE						NORMAL

/* MAC Adress */
#define MACADRESS					{ 0x00, 0x08, 0xDC, 0x01, 0x02, 0x05 };

#define GENERAL_NODE_WU_ADD			0xCCCC

/* AES SETTINGS */
#define AES_ENCRYPTION_KEY			{ 0x4D, 0x35, 0x32, 0x45, 0x4C, 0x39, 0x41, 0x42, 0x48, 0x32, 0x32, 0x53, 0x4C, 0x4A, 0x33, 0x4A }
#define AES_INIT_VECTOR				{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F }
#define AES_ENCRYPTION_PUBLIC_KEY	{ 0x44, 0x79, 0x62, 0x51, 0x63, 0x5B, 0x78, 0x4D, 0x2E, 0x2C, 0x51, 0x2E, 0x28, 0x56, 0x4B, 0x5A }

/* ENCRYPTION */
#define ETHERNET_ENCRYPTION			TRUE
#define NUM_SEND_RETRY				4
#define SWITCH_ANTENNA				true

/* Device modes */
#define NORMAL						1
#define TEST_WAKE_UP_TRANSMITTER	2
#define TEST_WAKE_UP_RECEIVER		3
#define SPECTRAL_ANALYSIS			4

// WAKE_UP_TEST mode config
#define TEST_WAKE_UP_ID				0xCACACA
#define TEST_WU_DELAY				5000 // in ms
#define TEST_WAKE_UP_OPTION			0xAA

/* WakeUp Address Length */
#define USE_8_BIT					1
#define USE_16_BIT					2
#define USE_24_BIT					3

// Choose WakeUp Address Length:
#define WU_ADDR_LENGTH				USE_24_BIT

#define USE_CSMA_CA					true
#define WDOG_ENABLED				false

/* LF frequency in Hz */
// Options: 19200, 125000
#define LF_FREQ						19200
#define HF_FREQ						869//915 or 869

/* SYSTEM SETTINGS */
#define CC1101_SPI_BAUDRATE	5000000ULL  // max. 6500000ULL
#define AS3933_SPI_BAUDRATE	5000000ULL  // max. 6000000ULL


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

/* Power profiles */

// OOK shaping
//#define PA_TABLE_LOW_POWER				{0x00,0x32,0x34,0x36,0x38,0x3A,0x3C,0x3F} // recommended for additional power amplifier
#define PA_TABLE_LOW_POWER				{0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37} // recommended for additional power amplifier
#define PA_TABLE_HIGH_POWER_BROADBAND	{0x00,0x03,0x0F,0xCF,0xCE,0xC9,0xC4,0xC0}
#define PA_TABLE_OOK_BROADBAND			{0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xC0} // not tested so far
#define PA_TABLE_HIGH_POWER_REDUCED_IF	{0xCF,0xCF,0xCE,0xCB,0xC7,0xC4,0xC1,0xC0} // recommended for regular wake-up
#define PA_TABLE_CARRIER				{0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0}


// OPTIONS: FSK
#define PA_FSK								{0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00}	// +12dBm
#define PA_FSK_LOW							{0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00}	// +0dBm
#define PA_FSK_ULTRA_LOW					{0x1D,0x00,0x00,0x00,0x00,0x00,0x00,0x00}	// -15dBm
