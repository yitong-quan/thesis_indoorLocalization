#ifndef UTILS_UART_H_
#define UTILS_UART_H_

#include "config.h"

#include <stdint.h>

#include "em_gpio.h"
#include "em_usart.h"

#include "TIMER.h"

// VARIABLES
char UART_receive[256];
uint16_t UART_kbs_bat_voltage;

// HW
#define UART_WAKE_1_PORT		gpioPortA
#define UART_WAKE_1_PIN			1
#define UART_WAKE_2_PORT		gpioPortF
#define UART_WAKE_2_PIN			2

// TIMEOUTS
#define	UART_KBS_ACK_TIMEOUT	1000  	// ms
#define	UART_KBS_STX_TIMEOUT	1000 	// ms
#define	UART_KBS_DATA_TIMEOUT	1000  	// ms

// CMD
#define UART_CMD_NUL			0x00
#define UART_CMD_STX			0x02
#define UART_CMD_ETX			0x03
#define UART_CMD_EOT			0x04
#define	UART_CMD_ENQ			0x05
#define UART_CMD_ACK			0x06
#define UART_CMD_DLE			0x10
#define UART_CMD_NAK			0x15
#define UART_CMD_TOUT			0x19

#define UART_KBS_OFFSET 		0x40

// UART ERR
#define UART_ERR_NAK			-3
#define UART_ERR_NUL			-2
#define UART_ERR_TOUT			-1

// REV 3 only
void UART_kbs_wake_enable(bool enable);
uint8_t UART_kbs_transmit(char CMD, char *data, uint16_t length);
int16_t UART_kbs_listen_for_data(char *data, uint16_t timeout_ms);
bool UART_kbs_listen_for_byte(uint8_t cmd, uint16_t timeout_ms);
uint16_t UART_kbs_get_bat_voltage();

/**
 * Sends out char array *str via USART1 (TX=PD7; RX=PD6)
 */
void UART_WriteString(char *str, int size);

/**
 * Sends out byte array *buf via USART1 (TX=PD7; RX=PD6)
 */
void UART_Write(uint8_t *buf, int size);

/**
 * Returns the length of available received UART Data.
 */
uint16_t UART_available(void);

/**
 * Received packets will be stored in global variable UART_receive
 */
void USART1_RX_IRQHandler(void);

/**
 * Enables UART RX Interrupt functionality.
 */
void UART_enableRxInterrupt(void);

#endif /* UTILS_UART_H_ */
