/*
 * checkCommand.c
 *
 *  Created on: 15.11.2016
 *      Author: Patrick Fehrenbach
 */
#include "checkCommand.h"
#include "helpfunctions.h"
#include "WorkModes.h"

extern char string_buffer[256]; 												// UART String Buffer

void check_command_setNUMBER(uint8_t *buffer, uint8_t *node_numbers)
{
	if(memcmp(buffer, "setNUMBER: ", 11) == 0)
	{
		long number_of_nodes = 0;
		uint8_t decimalstring[3] = {0};

		arraycpy(decimalstring, buffer, 11, 6);								// copy the HEX dezimal number (nodes) from the buffer to the dezimalstring array
		number_of_nodes = strtol((char *) decimalstring, NULL, 10);			// save the number of nodes as long
		*node_numbers = (uint8_t) number_of_nodes;

		sprintf(string_buffer, "Write down the 16 bit IDs of %d Nodes in HEX\r\n", *node_numbers);
		UART_WriteString(string_buffer, strlen(string_buffer));
		memset(string_buffer, '\0', sizeof(string_buffer));
	}
}

void check_command_setNODES(uint8_t *buffer, uint8_t *node_ids_array, uint8_t node_numbers)
{

	uint8_t idx = 0;

	if(memcmp(buffer, "setNODES: [0x", 13) == 0)
	{
		if (node_numbers == 0)
		{
			strcpy(string_buffer, "Please set Node number before you set the IDs\r\n");
			printMSG(string_buffer, strlen(string_buffer));
		}
		else
		{
			idx = 0;

			node_ids_array[idx++] = node_numbers;

			int i = 0;
			for(i = 0; i < node_numbers; i++)
			{
				long node_ids = 0;
				uint8_t node_high;
				uint8_t node_low;

				uint8_t hexstring[5] = {0};

				arraycpy(hexstring, buffer, 13 + (i * 8), 4);							// copy the HEX numbers (ID) from the buffer to the hexstring array
				node_ids = strtol((char *) hexstring, NULL, 16);						// save the HEX ID as long

				node_high = (uint8_t) (node_ids >> 8);
				node_low  = (uint8_t) node_ids;

				node_ids_array[idx++] = node_high;
				node_ids_array[idx++] = node_low;
			}
			strcpy(string_buffer, "Nodes IDs are set.\r\n");
			printMSG(string_buffer, strlen(string_buffer));
		}
	}
}


bool check_command_find(uint8_t *buffer)
{
	bool error = false;

	if (memcmp(buffer, "\r\n",2) == 0)
	{
		sendPacket(0x310D, AS_OPT_BYTE_NORMAL_MODE, NULL, 0, 1); //first para: Tag ID......last para: numberOfMeasurements
		/*
		char szNumbers[] = "0x310D";
		uint8_t tag_id;
		tag_id = strtol(szNumbers,NULL,0);
		sendPacket(tag_id, AS_OPT_BYTE_NORMAL_MODE, NULL, 0, 1); //first para: Tag ID......last para: numberOfMeasurements
		*/
	}

	if (memcmp(buffer, "find", 4) == 0)
	{
		long wake = 0;
		// Check if ID is in hex format
		if (memcmp(buffer, "find 0x", 7) == 0)
		{
			// potentially valid HEX command
			uint8_t hexstring[5] = {0};
			uint8_t decimalstring[3] = {0};

			arraycpy(hexstring, buffer, 7, 5);										// copy the HEX numbers (ID) from the buffer to the hexstring array
			wake = strtol((char *) hexstring, NULL, 16);							// save the HEX ID as long

			// check if ID within range
			if (wake < 65536)
			{
				long measurements = 0;

				arraycpy(decimalstring, buffer, 13, 6);								// copy the HEX numbers (ID) from the buffer to the hexstring array
				measurements = strtol((char *) decimalstring, NULL, 10);			// save the HEX ID as long

				if (measurements < 65536)
				{
					if(measurements == 0)
					{
						measurements = 25;
					}
					// valid wake address:
					sendPacket(wake, AS_OPT_BYTE_NORMAL_MODE, NULL, 0, measurements);
				}
				else // error:
				{
					strcpy(string_buffer, "NUMBER OF MEASUREMENTS not valid - must be < 65536\r\n");
					printMSG(string_buffer, strlen(string_buffer));
					error = true;
				}
			}
			else // error:
			{
				strcpy(string_buffer, "ID not valid - must be < 65536\r\n");
				printMSG(string_buffer, strlen(string_buffer));
				error = true;
			}
		}
		else
		{
			// Not supporting decimal format, throw an error:
			strcpy(string_buffer,
					"WRONG FORMAT\r\n");
			printMSG(string_buffer, strlen(string_buffer));
			error = true;
		}
	}
	return error;
}

bool check_command_find_node(uint8_t *buffer)
{

	bool error = false;
	if (memcmp(buffer, "findwNODE", 9) == 0)
	{
		long wake = 0;
		// Check if ID is in hex format
		if (memcmp(buffer, "findwNODE 0x", 12) == 0)
		{
			// potentially valid HEX command
			uint8_t hexstring[5] = {0};
			uint8_t decimalstring[3] = {0};

			arraycpy(hexstring, buffer, 12, 5);										// copy the HEX numbers (ID) from the buffer to the hexstring array
			wake = strtol((char *) hexstring, NULL, 16);							// save the HEX ID as long

			// check if ID within range
			if (wake < 65536)
			{
				long measurements = 0;

				arraycpy(decimalstring, buffer, 18, 6);								// copy the HEX numbers (ID) from the buffer to the hexstring array
				measurements = strtol((char *) decimalstring, NULL, 10);			// save the HEX ID as long

				if (measurements < 65536)
				{
					if(measurements == 0)
					{
						measurements = 25;
					}
					// valid wake address:
					sendPacket(wake, AS_OPT_BYTE_WU_NODE_TAG_MODE, NULL, 0, measurements);
				}
				else // error:
				{
					strcpy(string_buffer, "NUMBER OF MEASUREMENTS not valid - must be < 65536\r\n");
					printMSG(string_buffer, strlen(string_buffer));
					error = true;
				}
			}
			else // error:
			{
				strcpy(string_buffer, "ID not valid - must be < 65536\r\n");
				printMSG(string_buffer, strlen(string_buffer));
				error = true;
			}
		}
		else
		{
			// Not supporting decimal format, throw an error:
			strcpy(string_buffer,
					"WRONG FORMAT\r\n");
			printMSG(string_buffer, strlen(string_buffer));
			error = true;
		}
	}
	return error;
}

