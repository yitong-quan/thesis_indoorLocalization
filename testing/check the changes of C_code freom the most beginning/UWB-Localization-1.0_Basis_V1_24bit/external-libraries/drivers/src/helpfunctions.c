/*
 * helpfunctions.c
 *
 *  Created on: 15.11.2016
 *      Author: User
 */

#include "helpfunctions.h"


/*
 * Function: arraycpy
 * ----------------------------
 *   Copy an array from an start- to an endpoint
 *
 *   *dest: ptr to destination
 *   *src: ptr to source
 *   start: start position in source
 *   len: number of bytes
 *
 */

void arraycpy(uint8_t *dest, uint8_t *src, int start, int len)
{
	for (int i = 0; i < len; i++)
	{
		dest[i] = src[start + i];
	}
}

bool printMSG(char *buffer, int len)
{
	UART_WriteString(buffer, len);
	if (Ethernet_status(1) == SnSR_ESTABLISHED)
	{
#if ETHERNET_ENCRYPTION
		uint8_t length = Encrypt_AES128CBC((uint8_t *) buffer, len,
				AES_encryption_key, AES_initVector);
#else
		uint8_t length = (uint8_t) len;
#endif
		if (Ethernet_write(1, (uint8_t *) buffer, length) == 0)
		{
			// Something went wrong on transmitting data
			return true;
		}
	}
	memset(buffer, '\0', sizeof(buffer));
	return false;
}

bool switch_antenna(bool ant_use)
{
	if (ant_use)
	{
		return ANT2;
	}
	else
	{
		return ANT1;
	}
}

