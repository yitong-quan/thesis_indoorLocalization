/**************************************************************************//**
 * @file
 * @brief tableFinder-baseStation
 * @version
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_device.h"
#include "em_system.h"

#include "config.h"
#include "HWinit_loc.h"
#if MODE == NORMAL
#include "w5100.h"
#include "socket.h"
#include "dhcp.h"
#include "ethernet.h"
#endif
#include "CC1101.h"
#include "CC1190.h"
#include "RADIO.h"
#include "RADIO_CONFIG.h"
#include "RFPACKETHELPER.h"
#include "UART.h"
#include "CRC.h"
#include "RTC.h"
#include "EEPROM_handler.h"
#include "ANTENNA.h"
#include "WorkModes.h"
#include "TestModes.h"
#include "checkCommand.h"
#include "helpfunctions.h"

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/

/* PA table settings, see config.h */
uint8_t paTableWakeUp[8] = PA_TABLE_LOW_POWER;
uint8_t paTableData[8] = PA_FSK;

#define LEDPort gpioPortB
//#define red 13
#define green 14
//#define blue 11

//#define UART_receive_LENGTH 100

//char UART_receive[UART_receive_LENGTH];

/*Global variables*/
uint8_t MAC[] = MACADRESS
uint8_t Gateway[4] = { 0 };
uint8_t SubnetMask[4] = { 0 };
uint8_t IPAddress[4] = { 0 };
uint8_t buffer[256] = { 0 };
uint16_t port = 1;
uint8_t usedhcp = 0;

#if REVISION == BASE_REV_2
// Default set antenna 1
bool antUse = ANT1;
#endif
// echo on uart
int echo = 0;
bool error = false;
bool uartError = false;
int uartLength = 0;
int datLength = 0;

// BASIS VARIABLES

uint32_t MY_BASE_ID = 0x0001;  	// ID of this Basis

// SYSTEM SETTINGS
//#define CC1101_SPI_BAUDRATE	5000000ULL

// SYSTEM VARIABLES
struct radio_transfer xfer;  // Radio Transfer Handler
char string_buffer[256] = { 0 }; // UART String Buffer

/* AES VARIABLES */
uint8_t AES_decryption_key[16];  						// AES Decryption Key
uint8_t AES_encryption_key[] = AES_ENCRYPTION_KEY; // AES Encryption Key 128 bit
uint8_t AES_decryption_public_key[16];  				// AES Decryption Key
uint8_t AES_encryption_public_key[] = AES_ENCRYPTION_PUBLIC_KEY; // AES Encryption Key 128 bit
const uint8_t AES_initVector[] = AES_INIT_VECTOR; // Initialization vector used during CBC

uint8_t NODE_NUMBERS = 0;
//uint8_t NODE_IDs[10] = {1,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t NODE_IDs[10] = {0};
uint8_t idx = 0;
//uint8_t MAX_NUM_OF_TRIES = 3;
uint8_t MAX_NUM_OF_TRIES = 5;
// INTERRUPT ROUTINES DECLARATION
void TIMER1_IRQHandler(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

// Function declaration
void sendPacket(uint32_t wakeupid, uint8_t option, uint8_t *data, uint8_t len, uint16_t n_measurements);
void listenForResponse(uint8_t *buffer, uint32_t wakeupid, uint8_t msgType, uint8_t *len,
		uint8_t *act_packet, uint8_t *total_n_packet, uint32_t *tag_addr);
bool checkACK(uint32_t wakeupid);
uint8_t Encrypt_AES128CBC(uint8_t *buffer, uint8_t len, const uint8_t *key,
		const uint8_t *iv);
uint8_t Decrypt_AES128CBC(uint8_t *buffer, const uint8_t *key,
		const uint8_t *iv);
//void arraycpy(uint8_t *dest, uint8_t *src, int start, int len);
int stringToIP(uint8_t *buffer, uint8_t *ip);
void getID(uint8_t *buffer, uint32_t *id, uint8_t *comlen, uint8_t offset,
		bool isHEX);
uint8_t getBufferLength(uint8_t *buffer);
uint8_t getIdLength(uint8_t *buffer, uint8_t start);
uint8_t getIdSpace(uint8_t *buffer, uint8_t start);
void testComponents(void);

bool send_NODE_IDs(uint32_t wakeupid, uint8_t *buffer);

bool wake_up_function(uint32_t wakeupid, uint8_t option, uint16_t n_measurements);
void wake_up_function_without_ACK(uint32_t wakeupid, uint8_t option, uint16_t n_measurements);

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void) {
	/* Chip errata */
	CHIP_Init();

	// Use RC Oscillator (HF)
	CMU_OscillatorEnable(cmuOsc_HFRCO, true, false);
	CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
	CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

	// Use Crystal Osc (LF)
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_RTC, true);

	EEPROM_init();

	//==============================================================================
	// enter_DefaultMode_from_RESET
	//==============================================================================
	enter_DefaultMode_from_RESET();

	// RTC init
	RTC_init(1000);  // Init RTC, set to 1s
	TIMER_init(TIMER0, cmuClock_TIMER0, TIMER0_IRQn);
	TIMER_init_us(TIMER2, cmuClock_TIMER2, TIMER2_IRQn);

	SysTick_Delay_init();

	/* Initialize CC1101 RF Module */
	//cc1101_spi_init(CC1101_SPI_BAUDRATE);
	//cc1101_change_config_to(CC1101_WAKEUP_CONFIG, PLUS_12dBm);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn); /* Enable interrupt. */
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	radio_init(&xfer, TIMER1, cmuClock_TIMER1, TIMER1_IRQn);

	// Reads data from Flash and writes it into local variable
	//EEPROM_read_and_store_into(&MY_BASE_ID, AES_encryption_key, IPAddress,
		//	SubnetMask, Gateway, &port, &usedhcp);

	//Use a testmode
	#if MODE != NORMAL
		TESTMODE_enter(&xfer);
	#else
	// TODO: Add BASE_ID to DHCP name
	Dhcp_setID(&MY_BASE_ID);

	// Calculate (once) decryption key from the original key
	AES_DecryptKey128(AES_decryption_key, AES_encryption_key);
	AES_DecryptKey128(AES_decryption_public_key, AES_encryption_public_key);

#if REVISION == BASE_REV_2
	//switch to receive mode
	BASE_ANTENNA_switch(SKY_DEST_PE, PE_DEST_SKY, antUse);
#endif
	cc1190_receive();

	// Test all leds
	testComponents();

	// Status of the ethernet initialization
	int etherstatus = 0;

	// Config interrupt from W5100
	//GPIO_IntConfig(W5100IntPort, W5100IntPin, false, true, true);
	//GPIO_IntEnable(1 << W5100IntPin);

	// Print the Basis ID serial
	/*sprintf(string_buffer, "Basis ID: 0x%X (%d) \r\n", (int) MY_BASE_ID,			//Wieder rein machen
			(int) MY_BASE_ID);
	UART_WriteString(string_buffer, strlen(string_buffer));*/
	memset(string_buffer, '\0', sizeof(string_buffer));

	RTC_delay_ms(1500);

	if (usedhcp == 1) {
		 //Init ethernet connection with DHCP use
		etherstatus = Ethernet_dhcp_init(MAC, IPAddress, Gateway, SubnetMask);
	} else {
		// Init ethernet connection without DHCP use
		Ethernet_init(MAC, IPAddress, Gateway, SubnetMask);
		etherstatus = 1;
	}

	if (etherstatus)
	{
		// Start TCP server
		Ethernet_start(1, port);
		if (usedhcp == 1)
		{
			strcpy(string_buffer, "Network settings got from DHCP\r\n");
			UART_WriteString(string_buffer, strlen(string_buffer));
			memset(string_buffer, '\0', sizeof(string_buffer));
		}
		/*sprintf(string_buffer, "TCP-Server started on Port %d\r\n", port);
		UART_WriteString(string_buffer, strlen(string_buffer));
		memset(string_buffer, '\0', sizeof(string_buffer));
		sprintf(string_buffer, "IP-Adress: %d.%d.%d.%d\r\n", IPAddress[0], IPAddress[1], IPAddress[2], IPAddress[3]);
		UART_WriteString(string_buffer, strlen(string_buffer));
		memset(string_buffer, '\0', sizeof(string_buffer));
		sprintf(string_buffer, "Subnet Mask: %d.%d.%d.%d\r\n", SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3]);
		UART_WriteString(string_buffer, strlen(string_buffer));
		memset(string_buffer, '\0', sizeof(string_buffer));
		sprintf(string_buffer, "Gateway: %d.%d.%d.%d\r\n", Gateway[0], Gateway[1], Gateway[2], Gateway[3]);
		UART_WriteString(string_buffer, strlen(string_buffer));*/
	}
	else
	{
		strcpy(string_buffer, "DHCP Error!\r\n");
		UART_WriteString(string_buffer, strlen(string_buffer));
	}


	//UART_WriteString("Ready\r\n", strlen("Ready\r\n"));

	strcpy(string_buffer, "Please set all used Node IDs\r\n");
	error = printMSG(string_buffer, strlen(string_buffer));

	strcpy(string_buffer, "How many Nodes do you want to connect?\r\n");
	error = printMSG(string_buffer, strlen(string_buffer));


	UART_enableRxInterrupt();

	int length = 0;
	int etherLength = 0;
	uint8_t established = 0;
	uint8_t etherstat = 0;

	while (1)
	{
		//etherstat = Ethernet_status(1);													// Check Ethernet status

		if (etherstat == SnSR_ESTABLISHED) 												// When client is connected
		{
			if (!established)
			{
				established = 1;
				UART_WriteString("Client connected\r\n",
						strlen("Client connected\r\n"));
				sprintf((char*) buffer, "Welcome at Basis ID: 0x%X (%d) \r\n",
						(int) MY_BASE_ID, (int) MY_BASE_ID);
#if ETHERNET_ENCRYPTION
				uint8_t length = Encrypt_AES128CBC(buffer,
						strlen((char*) buffer), AES_encryption_key,
						AES_initVector);
#else
				uint8_t length = strlen((char*) buffer);
#endif
				Ethernet_write(1, buffer, length);
				memset(buffer, '\0', sizeof(buffer));
			}

			//Check if data is available
			etherLength = Ethernet_data_available(1);
			if (etherLength > 0)
			{
				length = etherLength;
				Ethernet_read(1, buffer, length);
#if ETHERNET_ENCRYPTION
				length = Decrypt_AES128CBC(buffer, AES_decryption_key,
						AES_initVector);
#endif

#if PRINT
				UART_WriteString((char *) buffer, length);
#endif
			}
		}

		// Close connection if client disconnected
		if (etherstat == SnSR_CLOSE_WAIT)
		{
			Ethernet_stop(1);
			UART_WriteString("Client disconnected\r\n",
					strlen("Client disconnected\r\n"));
			continue;
		}
		if (etherstatus != 0 && etherstat == SnSR_CLOSED)
		{
			Ethernet_start(1, port);
			established = 0;
		}

		if (uartError)
		{
			//strcpy(string_buffer, "WRONG COMMAND\r\n");
			//error = printMSG(string_buffer, strlen(string_buffer));
			uartError = false;
			continue;
		}

		if (uartLength)
		{
			length = uartLength;
			memcpy(buffer, UART_receive, length);
			memset(UART_receive, '\0', sizeof(UART_receive));
			if (echo) {
				// send back command
				UART_WriteString((char *) buffer, length);
			}
		}



/*****************************************************************************************************************************
 * Command received:																									   ***
 ****************************************************************************************************************************/
		if (length > 0)
		{
			length = 0;																// Reset the command length
			//NODE_NUMBERS = 1;
			check_command_setNUMBER(buffer, &NODE_NUMBERS);							// check command setNUMBER
			check_command_setNODES(buffer, NODE_IDs, NODE_NUMBERS);					// check command setNODES

			error = check_command_find(buffer);										// check command find

			/*if (error)
			{
				strcpy(string_buffer, "NACK\r\n");
				error = printMSG(string_buffer, strlen(string_buffer));
				error = false;
				uartLength = 0;
			}
			else
			{
				strcpy(string_buffer, "ACK\r\n");
				error = printMSG(string_buffer, strlen(string_buffer));
				error = false;
				uartLength = 0;
			}*/
			memset(buffer, '\0', sizeof(buffer));
		}
	}
#endif
}
#if MODE == NORMAL
/*
 * Function: sendPacket
 * ----------------------------
 *   Send to Identifier with different options
 *
 *   wakeupid: ID of the Identifier
 *   option: option byte
 *   *data: ptr to data array (Null if no data is needed)
 *   len: length of data to send
 *
 */

void sendPacket(uint32_t wakeupid, uint8_t option, uint8_t *data, uint8_t len, uint16_t n_measurements)
{
	error = false;

	memset(string_buffer, '\0', sizeof(string_buffer));

	uint8_t CC1101_receive[256] = { 0 };
	uint8_t act_packet = 0;
	uint8_t total_packets = 10;

	// Send Wakeup to all the NODES
	wake_up_function_without_ACK(GENERAL_NODE_WU_ADD, option, n_measurements);

	// Send Wakeup to Tag
	// Try to send the wakeup again if it fails
	if (wake_up_function(wakeupid, option, n_measurements))
	{
		return;
	}

	if (!send_NODE_IDs(wakeupid, NODE_IDs))						// send the node IDs to the Tag
	{
		return;
	}

	// Set CC1101 RF Module to Data Communication Configuration
	cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

	uint32_t Tag_addr = 0;
	uint16_t i = 0, n = 0;
	uint8_t num = 0;
	uint8_t CC1101_full_data[800] = { 0 };
	uint8_t uintUWB_data[800] = { 0 };
	uint16_t NODE_ADDRESS = 0x00;
	float UWB_data[200] = { 0 };
	memset(CC1101_receive, 0, 226);

	// Find the TAG
	for (i = 1; i <= total_packets; i++)
	{
		listenForResponse(CC1101_receive, wakeupid, CC_OPT_BYTE_MSG_IDENTIFIER_TO_BASIS, &len, &act_packet, &total_packets, &Tag_addr);
		if (error)
		{
			return;
		}
		arraycpy(&CC1101_full_data[(act_packet - 1) * 226], CC1101_receive, 0, 226);
	}

	memset(UWB_data, 0, 200);

	for (num = 0; num < NODE_NUMBERS; num++)
	{
		arraycpy(uintUWB_data, CC1101_full_data, num*((n_measurements * 4) + 2), ((n_measurements * 4) + 2));
		NODE_ADDRESS = (((uint16_t) uintUWB_data[0]) << 8 | ((uint16_t) uintUWB_data[1]));

		// remove IDs
		for (i = 0; i < (n_measurements * 4) + 2; i++)
		{
			if (i < (n_measurements * 4))
			{
				uintUWB_data[i] = uintUWB_data[i+2];
			}
			else
			{
				uintUWB_data[i] = 0x00;
			}
		}

		float *UWB_float_data_pointer = (float *)uintUWB_data;
		for(uint16_t n = 0; n < n_measurements; n++)
		{
			UWB_data[n] = *UWB_float_data_pointer;
			UWB_float_data_pointer++;
		}

		if(!NODE_ADDRESS)
		{
			//disable for nicer presentation. TODO, should be back. Yitong
			//UART_WriteString("NODE is out of range, Wake up not possible\r\n", sizeof("NODE is out of range, Wake up not possible\r\n"));
			sprintf(string_buffer, "no data recieved! \r\n");
			error = printMSG(string_buffer, strlen(string_buffer));
		}
		else
		{
			//sprintf(string_buffer,"0x%X, ",(int) NODE_ADDRESS);
			////sprintf(string_buffer,"Received Packet from NODE: 0x%X \r\n",(int) NODE_ADDRESS);
			//error = printMSG(string_buffer, strlen(string_buffer));

			for (n = 0; n < n_measurements; n++)
			{

				//sprintf(string_buffer, "%ld \r\n", (long int) (UWB_data[n] * 100));
				//error = printMSG(string_buffer, strlen(string_buffer));

//TODO: buffer<<nodeID1<<dist,dist,dist...<<nodeID2<<dist,dist,dist...<<nodeID3...    buffer>>out

				if(NODE_ADDRESS == 0x1C1C)
				{
					//sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (89.73 * UWB_data[n] + 25.4));		// y = 0.9925  * x - 5.205
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n] * 0.9734 - 159.5));
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n]));
					error = printMSG(string_buffer, strlen(string_buffer));
				}
				if(NODE_ADDRESS == 0x2020)
				{
					//sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (90.42 * UWB_data[n] + 5.531));		// y = 0.9993  * x - 0.24217
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n] * 1.034 -291.3));		// y = 0.9993  * x - 0.24217error = printMSG(string_buffer, strlen(string_buffer));
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n]));
					error = printMSG(string_buffer, strlen(string_buffer));
				}
				if(NODE_ADDRESS == 0x3E3E)
				{
					//sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (91.91 * UWB_data[n] + 23.44));		// y = 1.00547 * x - 14.103
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n] * 0.9893 -218.1));		// y = 1.00547 * x - 14.103
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n]));
					error = printMSG(string_buffer, strlen(string_buffer));
				}
				if(NODE_ADDRESS == 0x4D4D)
				{
					//sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (89.84 * UWB_data[n] + 21.37));		// y = 1.01452 * x - 8.1253
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n] * 1.052 -688.6));		// y = 1.01452 * x - 8.1253
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n]));
					error = printMSG(string_buffer, strlen(string_buffer));
				}
				if(NODE_ADDRESS == 0x5A5A)
				{
					//sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1.0 * UWB_data[n] * 100 - 0));		// y = 1.01015 * x - 4.9066
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n] * 1.017 -339));
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n]));
					error = printMSG(string_buffer, strlen(string_buffer));
				}
				if(NODE_ADDRESS == 0x6E6E)
				{
					//sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (91.74 * UWB_data[n] + 15.89));		// y = 0.9993  * x - 0.24217
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n] * 1.074 -796.4));		// y = 0.9993  * x - 0.24217
					sprintf(string_buffer, "0x%X , %ld \r\n", (int) NODE_ADDRESS, (long int) (1000 * UWB_data[n]));
					error = printMSG(string_buffer, strlen(string_buffer));
				}
			}
		}
	}
}

void wake_up_function_without_ACK(uint32_t wakeupid, uint8_t option, uint16_t n_measurements)
{
	for (int i = 0; i < 4; i++)
	{
#if PRINT
		sprintf(string_buffer,
		"Sending WakeUp to 0x%X with Antenna: %d and Option: 0x%X ... \r\n",
		(int) wakeupid, (int) antUse, (int) option);
		printMSG(string_buffer, strlen(string_buffer));
#endif

		// Set CC1101 RF Module to WakeUp Configuration
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		//cc1101_change_config_to(CC1101_WAKEUP_CONFIG, PLUS_12dBm);
		cc1101_change_config_to(CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);

		// switch to send wakeup mode (use CC1190)
		BASE_ANTENNA_switch(SKY_DEST_CC1190, PE_DEST_CC1190, antUse);

		cc1190_send();

		// Send WakeUp Packet
		cc1101_send_wakeup_packet_16bit_addr(&xfer, wakeupid, MY_BASE_ID, option, n_measurements);

		antUse = switch_antenna(antUse);
	}

}

bool wake_up_function(uint32_t wakeupid, uint8_t option, uint16_t n_measurements)
{
	for (int i = 0; i < NUM_SEND_RETRY; i++)
	{
#if PRINT
		sprintf(string_buffer,
		"Sending WakeUp to 0x%X with Antenna: %d and Option: 0x%X ... \r\n",
		(int) wakeupid, (int) antUse, (int) option);
		printMSG(string_buffer, strlen(string_buffer));
#endif
		// Set CC1101 RF Module to WakeUp Configuration
		cc1101_spi_init(CC1101_SPI_BAUDRATE);
		//cc1101_change_config_to(CC1101_WAKEUP_CONFIG, PLUS_12dBm);
		cc1101_change_config_to(CC1101_WAKEUP_CONFIG_NARROW_BAND, paTableWakeUp);

		// switch to send wakeup mode (use CC1190)
		BASE_ANTENNA_switch(SKY_DEST_CC1190, PE_DEST_CC1190, antUse);

		cc1190_send();
		// Send WakeUp Packet
		cc1101_send_wakeup_packet_16bit_addr(&xfer, wakeupid, MY_BASE_ID, option, n_measurements);

		// switch to receive / transmit data mode
		BASE_ANTENNA_switch(SKY_DEST_PE, PE_DEST_SKY, antUse);

		cc1190_receive();

		if (!checkACK(wakeupid))									// Check if ack is received
		{
			error = true;
	#if SWITCH_ANTENNA
			antUse = switch_antenna(antUse);
	#endif
		}
		else
		{
			error = false;
			break;
		}

	}
	return error;
}


/*
 * Function: listenForResponse
 * ----------------------------
 *   wait for response within 3000 microseconds
 *
 *   *buffer: ptr to byte array for received data
 *   wakeupid: identifier ID
 *   msgType: type of message
 *   *len: ptr to length of data
 *
 */

void listenForResponse(uint8_t *buffer, uint32_t wakeupid, uint8_t msgType, uint8_t *len,
		uint8_t *act_packet, uint8_t *total_n_packet, uint32_t *tag_addr) {

	RTC_start(750); // Patrick has set 11000, I think it is too much. Yitong

	while (!RTC_TIMEOUT) {
		memset(buffer, 0x00, sizeof(buffer));
		// Receive Data from Identifier
		radio_receive_packet(&xfer, buffer, len, 800);		//Patrick set the last para:3000. I think it's too large. Yitong 0x1533720 >>	//3195//1000

		// Decrypt received packet (AES128CBC)
		RFPacket_decrypt_AES128CBC(buffer, AES_decryption_key, AES_initVector);

		*tag_addr = (((uint32_t) buffer[0]) << 16
				| ((uint32_t) buffer[1]) << 8 | buffer[2]);

		*act_packet = buffer[8];
		*total_n_packet = buffer[9];

		if (!RFPacket_check_and_remove_header_DWM1000(buffer, &wakeupid, &MY_BASE_ID,
				&msgType)) {
			// Packet is not addressed to me or has the wrong type
			continue;
		}

		// Set CC1101 RF Module to Data Communication Configuration
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);

		// Acknowledge reception of data from identifier
		cc1101_ack_reception(&xfer, &MY_BASE_ID, tag_addr,
				AES_encryption_key, AES_initVector);

		// Set CC1101 RF Module to Data Communication Configuration
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
		return;
	}
	error = true;
}

/*
 * Function: checkACK
 * ----------------------------
 *   check if ack is received within 500 ms
 *
 *   wakeupid: identifier ID
 *
 *   returns: state of ack
 */

bool checkACK(uint32_t wakeupid) {
// Check that Identifier has received wake up
	bool ack = false;
	// Wait for ACK
	RTC_start(180);	// 500 //180 Yitong
	while (!RTC_TIMEOUT) {
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
		ack = cc1101_check_ack(&xfer, &wakeupid, &MY_BASE_ID,
				AES_decryption_key, AES_initVector, 180);		//500 //200 Yitong
		if (ack) {
			break;
		}
	}
	if (RTC_TIMEOUT) {
#if PRINT
		UART_WriteString("TIMEOUT!\r\n", sizeof("TIMEOUT!\r\n"));
#endif
		error = true;
	}
#if PRINT
	if (ack)
	{
		strcpy(string_buffer, "ACK!\r\n");
		printMSG(string_buffer, strlen(string_buffer));
	}
	else
	{
		strcpy(string_buffer, "NOT ACK!\r\n\r\n");
		printMSG(string_buffer, strlen(string_buffer));
	}
#endif
	return ack;
}

/*
 * Function: Encrypt_AES128CBC
 * ----------------------------
 *   encrypt message with AES128CBC
 *
 *   *buffer: ptr to byte array with msg
 *   *key: ptr to encryptionkey
 *   *iv: ptr to initial vector
 *
 *   returns: length of the encrypted msg
 */

uint8_t Encrypt_AES128CBC(uint8_t *buffer, uint8_t len, const uint8_t *key,
		const uint8_t *iv) {
	uint8_t length = len;
	len = ((len - 1) | 15) + 1;  // Must be a multiple of 16

	uint8_t AESBuffer[len];
	memset(AESBuffer, 0x00, len);

	memcpy(AESBuffer, buffer, length);

	/* Encrypt data in AES_128 CBC */
	AES_CBC128(AESBuffer, AESBuffer, len, key, iv, true);

	memset(buffer, 0x00, sizeof(buffer));
	memcpy(buffer, AESBuffer, len);

	return len;
}

/*
 * Function: Decrypt_AES128CBC
 * ----------------------------
 *   Decrypt message with AES128CBC
 *
 *   *buffer: ptr to byte array with msg
 *   *key: ptr to decryptionkey
 *   *iv: ptr to initial vector
 *
 *   returns: length of the decrypted msg
 */

uint8_t Decrypt_AES128CBC(uint8_t *buffer, const uint8_t *key,
		const uint8_t *iv) {
//uint8_t len = getBufferLength(buffer);

	uint8_t len = 0;
	while (!(buffer[len] == '\0' && buffer[len + 1] == '\0')) {
		len++;
	}

	AES_CBC128(buffer, buffer, len, key, iv, false);
	buffer[len] = '\r';
	buffer[len + 1] = '\n';

	return len;
}

/*
 * Function: stringToIP
 * ----------------------------
 *   Converts an IP String to an 4 byte array
 *
 *   *buffer: ptr to stringbuffer with IP
 *   *ip: ptr to 4 byte array for IP
 *
 *   returns: number of bytes
 */

int stringToIP(uint8_t *buffer, uint8_t *ip) {
	char iptemp[4] = { 0 };
	char number[4] = { 0 };
	int pos = 0;
	int num = 0;
	int ret = 0;
	while (buffer[pos + 1] != '\n' && num < 4) {
		if (buffer[pos] == '.') {
			iptemp[ret] = atoi(number);
			pos++;
			ret++;
			num = 0;
			memset(number, '\0', 4);
		}
		if (buffer[pos] >= '0' && buffer[pos] <= '9') {
			number[num] = buffer[pos];
			pos++;
			num++;
		} else {
			return 0;
		}
	}
	if (memcmp(number, "\r", 1) == 0) {
		return 0;
	}
	iptemp[ret] = atoi(number);
	ret++;

	for (int i = 0; i < 4; i++) {
		ip[i] = iptemp[i];
	}
	return ret;
}

/*
 * Function: getID
 * ----------------------------
 *   Counts the bytes until \r\n
 *
 *   *buffer: ptr to byte array
 *   *id: ptr to ID
 *   *comlen: ptr with length of command in buffer
 *   offset: offset i.e. for 0x at HEX-ID
 *   isHEX: is the ID in HEX-format
 *
 */

void getID(uint8_t *buffer, uint32_t *id, uint8_t *comlen, uint8_t offset,
		bool isHEX) {

	uint8_t idLen = getIdLength(buffer, *comlen);
	// ID is in hex format
	if (isHEX) {
		uint8_t hexstring[7] = { 0 };
		arraycpy(hexstring, buffer, *comlen + offset, idLen);
		*comlen = idLen + (*comlen + offset);
		*id = (uint32_t) strtol((char *) hexstring, NULL, 16);
	} else { // ID is in decimal format
		uint8_t wakeupid[9] = { 0 };
		arraycpy(wakeupid, buffer, *comlen, idLen);
		*comlen = idLen + *comlen;
		*id = (uint32_t) atoi((char *) wakeupid);
	}
}

/*
 * Function: getBufferLength
 * ----------------------------
 *   Counts the bytes until \r\n
 *
 *   *buffer: ptr to byte array
 *
 *   returns: number of bytes in buffer
 */

// Counts the bytes until \r\n
uint8_t getBufferLength(uint8_t *buffer) {
	uint8_t len = 0;
	while (!(buffer[len] == '\r' && buffer[len + 1] == '\n')) {
		len++;
		if (len == 255) {
			return 0;
		}
	}
	return len + 2;
}

/*
 * Function: getIdLength
 * ----------------------------
 *   Counts the bytes until \0, [space], \r
 *
 *   *buffer: ptr to byte array with ID
 *
 *   returns: number of bytes in buffer
 */

uint8_t getIdLength(uint8_t *buffer, uint8_t start) {
	uint8_t len = 0;
	while (!((buffer[start + len] == '\0') || buffer[start + len] == ' '
			|| buffer[start + len] == '\r')) {
		len++;
		if (len == 255) {
			return 0;
		}

	}
	return len;
}

uint8_t getIdSpace(uint8_t *buffer, uint8_t start) {
	uint8_t len = 0;
	while (!(buffer[start + len] == ' ') && (len < 255)) {
		len++;
	}
	return len;
}

/*
 * Function: testComponents
 * ----------------------------
 *   Inital test of the components
 *
 */

void testComponents(void) {
#if	REVISION == BASE_REV_1
// Show all LED color
	LED_setLED(COL_RED);
	RTC_delay_ms(250);
	LED_clearLED();
	LED_setLED(COL_GREEN);
	RTC_delay_ms(250);
	LED_clearLED();
	LED_setLED(COL_BLUE);
	RTC_delay_ms(250);
	LED_clearLED();
#elif	REVISION == BASE_REV_2
// Show all LED color
	LED_setLED(COL_RED);
	RTC_delay_ms(250);
	LED_clearLED();
	LED_setLED(COL_GREEN);
	RTC_delay_ms(250);
	LED_clearLED();
#endif
}

bool send_NODE_IDs(uint32_t wakeupid, uint8_t *buffer)
{
	uint8_t CC1101_send[256] = { 0 };
	uint8_t num_of_tries = 0;
	bool acknowledged = true;
	/*
	//for debug, Yitong TODO: check ok or not
	if (idx == 1){
		idx = 17; //17 == 1+16
	}
	*/
	// Build Header
	RFPacket_build_header(CC1101_send, buffer, &MY_BASE_ID, &wakeupid, CC_OPT_BYTE_DAT_BASIS_TO_IDENTIFIER, &idx);

	// Add length byte for CC
	RFPacket_add_cc_length(CC1101_send, &idx);

	// Encrypt send packet (AES128CBC)
	RFPacket_encrypt_AES128CBC(CC1101_send, &idx, AES_encryption_key, AES_initVector);

	//for debug, Yitong TODO: check ok or not
	if (idx > 225){
		idx = 17;
	}

	// Send packet
	while (num_of_tries < MAX_NUM_OF_TRIES)
	{
		// Send Data to Basis Station
		RTC_start(50);
		// Send out packet if channel is free
		cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
		if (radio_send_packet_use_CSMA_CA(&xfer, CC1101_send, idx))
		{
			// Check if Base ACK my packet
			cc1101_change_config_to(CC1101_DATA_38kBaud_CONFIG, paTableData);
			acknowledged = cc1101_check_ack(&xfer, &wakeupid, &MY_BASE_ID, AES_decryption_key, AES_initVector, 75);  //85?

			if (acknowledged) 					// ACK!
			{
				break; 							// ACK! -> leave while
			}
			num_of_tries++;
		}
		while(!RTC_TIMEOUT);
	}
	return acknowledged;
}

//
void USART1_RX_IRQHandler(void) {
	if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
		int i = 0;
		memset(UART_receive, '\0', sizeof(UART_receive));
		bool getData = false;
		uint8_t comLength = 0;
		UART_receive[i] = USART_Rx(USART1 );
		uint32_t rounds = 0;
		while (i < 256 && rounds < 140000) {
			rounds++;
			if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
				i++;
				UART_receive[i] = USART_Rx(USART1 );       // store the data
				if (i < 11 && !getData
						&& memcmp(UART_receive, "transmit", 8) == 0) {
					getData = true;
				}
				if (getData && i > 10 && i < 18 && UART_receive[i] == ' ') {
					// Check if ID is in hex format
					if (memcmp(UART_receive, "transmit 0x", 11) == 0) {
						comLength = getIdLength((uint8_t*) UART_receive, 11)
								+ 11;
					} else { // ID is in decimal format
						char wakeupid[9] = { 0 };
						int pos = 0;
						while (pos < 8 && UART_receive[9 + pos] != ' ') {
							wakeupid[pos] = UART_receive[9 + pos];
							pos++;
						}
						comLength = getIdLength((uint8_t *) wakeupid, 0) + 9;
					}
				}
				if (getData && comLength != 0 && i > comLength
						&& i < comLength + 6 && UART_receive[i] == ' ') {
					uint8_t tmpLength[4] = { 0 };
					datLength = getIdLength((uint8_t *) UART_receive,
							comLength + 1);
					arraycpy(tmpLength, (uint8_t *) UART_receive, comLength + 1,
							datLength);
					comLength += datLength;
					datLength = atoi((char *) tmpLength);
				}

				if (getData) {
					// TODO: Check
					if (datLength != 0 && comLength != 0
							&& i >= comLength + 1 + datLength) {
						//uartLength = i + 1;
						//break;
						getData = false;
					}
				} else {
					if (UART_receive[i - 1] == '\r'
							&& UART_receive[i] == '\n') {
						uartLength = i + 1;
						break;
					}
				}
			}
			uartLength = 0;
			if (rounds == 140000) {
				uartError = true;
			}
		}
	}
	/* Clear RXDATAV interrupt */
	USART_IntClear(USART1, USART_IF_RXDATAV);
}
#endif

// Radio timeout timer
void TIMER1_IRQHandler(void) {
	__disable_irq();
	radio_handle_timeout(&xfer);
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	__enable_irq();
}

// CC1101 GDO0 Interrupt
void GPIO_EVEN_IRQHandler(void) {
	if (GPIO_IntGetEnabled() & (1 << CC1101_GDO0_PIN)) {
		radio_handle_transfer_end(&xfer);
		/* Clear interrupt flag. */
		GPIO_IntClear(1 << CC1101_GDO0_PIN);
	}
#if MODE == NORMAL
	if (GPIO_IntGetEnabled() & (1 << W5100IntPin)) {

		GPIO_IntClear(1 << W5100IntPin);
	}
#endif
}

// CC1101 GDO2 Interrupt
void GPIO_ODD_IRQHandler(void) {
	if (GPIO_IntGetEnabled() & (1 << CC1101_GDO2_PIN)) {
		radio_handle_transfer_threshold(&xfer);
		/* Clear interrupt flag. */
		GPIO_IntClear(1 << CC1101_GDO2_PIN);
	}
}

