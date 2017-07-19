#include "UART.h"

//
void UART_WriteString(char *str, int size) {
	int i;
	for (i = 0; i < size; i++) {
		if (str[i] == '\0')
			return;
		USART_Tx(USART1, str[i]); // send character
	}

	while (!(USART1 ->STATUS & USART_STATUS_TXC))
		;
}

void UART_Write(uint8_t *buf, int size) {
	int i;
	for (i = 0; i < size; i++) {
		USART_Tx(USART1, buf[i]); // send byte
	}

	while (!(USART1 ->STATUS & USART_STATUS_TXC))
		;
}

//
uint16_t UART_available(void) {
	uint16_t length = 0;
	while (UART_receive[length] != '\0') {
		length++;
	}

	return length;
}

//
void UART_enableRxInterrupt(void) {
	USART_IntClear(USART1, _USART_IF_MASK);  // clear all USART interrupt flags
	USART1 ->IEN = USART_IEN_RXDATAV;
	NVIC_EnableIRQ(USART1_RX_IRQn);
	NVIC_ClearPendingIRQ(USART1_TX_IRQn); // clear pending TX interrupt flag in NVIC
}

/*
 *  KBS UART PROTOCOL FUNCTIONS
 */

//
void UART_kbs_wake_enable(bool on_off) {
	/* Pin PF2 is configured to Output enabled */
	GPIO ->P[5].MODEL = (GPIO ->P[5].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
			| GPIO_P_MODEL_MODE2_PUSHPULL;
	GPIO_PinOutSet(UART_WAKE_2_PORT, UART_WAKE_2_PIN);

	GPIO_IntDisable(1 << UART_WAKE_1_PIN);
	GPIO_IntConfig(UART_WAKE_1_PORT, UART_WAKE_1_PIN, false, true, on_off);
}

//
uint8_t UART_kbs_transmit(char CMD, char *data, uint16_t length) {
	uint16_t crc = 0x00;

	// Bugfix for power up reset problem!
	char received_char;
	if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
		received_char = USART_Rx(USART1 );
	}
	received_char = 0xFF;

	// Set Wake Up Signal
	GPIO_PinOutClear(UART_WAKE_2_PORT, UART_WAKE_2_PIN);

	// Listen for ACK, DLE, Timeout
	TIMER_start(TIMER0, UART_KBS_ACK_TIMEOUT);
	while (received_char != UART_CMD_ACK && received_char != UART_CMD_DLE && !TIMER_TIMEOUT) {
		if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
			received_char = USART_Rx(USART1 );
		}
	}
	if (TIMER_TIMEOUT) {
		TIMER_stop(TIMER0 );
		// NOT ACK -> Timeout
		GPIO_PinOutSet(UART_WAKE_2_PORT, UART_WAKE_2_PIN);
		return UART_CMD_TOUT;
	}
	TIMER_stop(TIMER0 );

	if (received_char == UART_CMD_DLE) {
		// NOT ACK -> DLE
		GPIO_PinOutSet(UART_WAKE_2_PORT, UART_WAKE_2_PIN);
		return UART_CMD_DLE;
	}

	// STX
	USART_Tx(USART1, UART_CMD_STX);

	// CMD
	USART_Tx(USART1, CMD);
	crc = AddToCrc16(crc, CMD);

	// DATA
	uint16_t i;
	for (i = 1; i < length; i++) {
		if (data[i] <= UART_CMD_ENQ) {
			USART_Tx(USART1, UART_CMD_ENQ);
			//crc = AddToCrc16(crc, UART_CMD_ENQ);
			USART_Tx(USART1, (data[i] + UART_KBS_OFFSET));
			//crc = AddToCrc16(crc, (data[i] + UART_KBS_OFFSET));
			crc = AddToCrc16(crc, (data[i]));
		} else {
			USART_Tx(USART1, data[i]);
			crc = AddToCrc16(crc, data[i]);
		}
	}

	// EOT
	USART_Tx(USART1, UART_CMD_EOT);

	// CRC
	USART_Tx(USART1, (crc & 0xFF));
	USART_Tx(USART1, (crc >> 8));

	// ETX
	USART_Tx(USART1, UART_CMD_ETX);

	// Listen for ACK, NUL, Timeout
	received_char = 0xFF;
	TIMER_start(TIMER0, UART_KBS_ACK_TIMEOUT);
	while (received_char != UART_CMD_ACK && received_char != UART_CMD_DLE && received_char != UART_CMD_NAK && received_char != UART_CMD_NUL && !TIMER_TIMEOUT) {
		if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
			received_char = USART_Rx(USART1 );
		}
	}
	if (TIMER_TIMEOUT) {
		TIMER_stop(TIMER0 );
		// NOT ACK -> Timeout
		GPIO_PinOutSet(UART_WAKE_2_PORT, UART_WAKE_2_PIN);
		return UART_CMD_TOUT;
	}
	TIMER_stop(TIMER0 );

	if (received_char == UART_CMD_NAK || received_char == UART_CMD_NUL || received_char == UART_CMD_DLE) {
		// NOT ACK -> Null or Nak or DLE
		GPIO_PinOutSet(UART_WAKE_2_PORT, UART_WAKE_2_PIN);
		return received_char;
	}

	// Clear Wake Up Signal
	GPIO_PinOutSet(UART_WAKE_2_PORT, UART_WAKE_2_PIN);
	return UART_CMD_ACK;
}

//
bool UART_kbs_listen_for_byte(uint8_t cmd, uint16_t timeout_ms) {
	char received_char = 0x00;
	TIMER_start(TIMER0, timeout_ms);

	while (received_char != cmd && !TIMER_TIMEOUT) {
		// Poll UART
		if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
			received_char = USART_Rx(USART1 );
		}
	}

	if (TIMER_TIMEOUT) {
		TIMER_stop(TIMER0 );
		return false;
	}

	TIMER_stop(TIMER0 );
	return true;
}

//
int16_t UART_kbs_listen_for_data(char *data, uint16_t timeout_ms) {
	uint16_t length = 0, crc = 0;
	char received_char = 0x00;

	uint8_t received_crc[2] = { 0 };
	uint8_t i = 0;

	// Listen until EOT, if ENQ -> remove ENQ and subtract offset 0x40
	bool catch_data = true, subtract_offset = false;
	TIMER_start(TIMER0, timeout_ms);
	while (received_char != UART_CMD_ETX && !TIMER_TIMEOUT) {
		if (USART1 ->STATUS & USART_STATUS_RXDATAV) {
			received_char = USART_Rx(USART1 );
			if (received_char == UART_CMD_EOT) {
				catch_data = false;
				continue;
			}
			if (catch_data) {
				if (subtract_offset) {
					data[length++] = (received_char - UART_KBS_OFFSET);
					crc = AddToCrc16(crc, (received_char - UART_KBS_OFFSET));
					subtract_offset = false;
				} else {
					if (received_char == UART_CMD_ENQ) {
						subtract_offset = true;
					} else {
						data[length++] = received_char;
						crc = AddToCrc16(crc, received_char);
					}
				}
			} else {
				if (i < 2) {
					received_crc[i++] = received_char;
				} else {
					break;
				}
			}
		}
	}

	if (TIMER_TIMEOUT) {
		// No Data received -> NUL
		TIMER_stop(TIMER0 );
		return UART_ERR_NUL;
	}
	TIMER_stop(TIMER0 );

	// Check CRC
	uint16_t received_checksum = (uint16_t) received_crc[1] << 8 | received_crc[0];
	if (crc != received_checksum) {
		// CRC failed -> NAK
		return UART_ERR_NAK;
	} else {
		// Everything OK -> return length
		return length;
	}
}


uint16_t UART_kbs_get_bat_voltage() {
	UART_kbs_transmit('X', "", 0);
	UART_kbs_bat_voltage = 0;

	// Bat voltage will be received in Interrupt
	TIMER_start(TIMER0, 50);
	while(!UART_kbs_bat_voltage && !TIMER_TIMEOUT);

	#if USE_ADC_IF_NO_UART_BAT_RECEIVED
	if (!UART_kbs_bat_voltage) {
		UART_kbs_bat_voltage = ADC_get_bat_voltage();
	}
	#endif

	return UART_kbs_bat_voltage;
}
