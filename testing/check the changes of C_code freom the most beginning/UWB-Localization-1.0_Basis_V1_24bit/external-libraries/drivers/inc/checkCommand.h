/*
 * checkCommand.h
 *
 *  Created on: 15.11.2016
 *      Author: Patrick Fehrenbach
 */

#ifndef CHECKCOMMAND_H_
#define CHECKCOMMAND_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "UART.h"

void check_command_setNUMBER(uint8_t *buffer, uint8_t *node_numbers);

void check_command_setNODES(uint8_t *buffer, uint8_t *node_ids, uint8_t node_numbers);

bool check_command_find(uint8_t *buffer);

bool check_command_find_node(uint8_t *buffer);



#endif /* CHECKCOMMAND_H_ */
