/*
 * helpfunctions.h
 *
 *  Created on: 15.11.2016
 *      Author: User
 */

#ifndef HELPFUNCTIONS_H_
#define HELPFUNCTIONS_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "w5100.h"
#include "ethernet.h"
#include "UART.h"
#include "ANTENNA.h"

void arraycpy(uint8_t *dest, uint8_t *src, int start, int len);

bool printMSG(char *buffer, int len);

bool switch_antenna(bool ant_use);

#endif /* HELPFUNCTIONS_H_ */
