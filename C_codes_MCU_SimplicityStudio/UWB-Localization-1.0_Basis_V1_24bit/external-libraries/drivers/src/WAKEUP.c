#include "wakeup.h"

bool manchester = false;

void wakeup_add_value(uint8_t * buffer, uint16_t * position, uint8_t value,
		uint16_t amount) {
	uint16_t k = 0;
	for (; k < amount; k++) {
		buffer[position[0]++] = value;
	}
}

void wakeup_add_zero(uint8_t * buffer, uint16_t * position) {
	wakeup_add_value(buffer, position, LF_LOGIC_ZERO, LENGTH_WU_BIT_CC);
	wakeup_add_value(buffer, position, LF_LOGIC_ONE, LENGTH_WU_BIT_CC);
}

void wakeup_add_one(uint8_t * buffer, uint16_t * position) {
	wakeup_add_value(buffer, position, LF_LOGIC_ONE, LENGTH_WU_BIT_CC);
	wakeup_add_value(buffer, position, LF_LOGIC_ZERO, LENGTH_WU_BIT_CC);
}

uint16_t wakeup_build_packet_8bit(uint8_t address, uint8_t *buffer, uint8_t *payload,
		uint16_t payload_length) {

	/* build the wake-up message */
	uint16_t position = 0;
	uint8_t count;

	/* Carrier burst (for calibration). */
	wakeup_add_value(buffer, &position, LF_LOGIC_ONE, LENGTH_CB_CC);

	/* Separation bit */
	wakeup_add_value(buffer, &position, LF_LOGIC_ZERO, LENGTH_WU_BIT_CC);


	/* Preamble. */
	for (count = 0; count < LENGTH_PREAMBLE; count++) {
		wakeup_add_one(buffer, &position);
	}

	/* The address */
	for (count = 0; count < 8; count++) {
		if ((address & (1 << (7 - count)))) {
			wakeup_add_one(buffer, &position);
		} else {
			wakeup_add_zero(buffer, &position);
		}
	}

	wakeup_add_payload(&position, buffer, payload, payload_length);
	return position;
}

uint16_t wakeup_build_packet_16bit(uint16_t address, uint8_t *buffer, uint8_t *payload,
		uint16_t payload_length) {

	/* build the wake-up message */
	uint16_t position = 0;
	uint8_t count;

	/* Carrier burst (for calibration). */
	wakeup_add_value(buffer, &position, LF_LOGIC_ONE, LENGTH_CB_CC);

	/* Separation bit */
	wakeup_add_value(buffer, &position, LF_LOGIC_ZERO, LENGTH_WU_BIT_CC);


	/* Preamble. */
	for (count = 0; count < LENGTH_PREAMBLE; count++) {
		wakeup_add_one(buffer, &position);
	}

	/* The address */
	for (count = 0; count < 16; count++) {
		if ((address & (1 << (15 - count)))) {
			wakeup_add_one(buffer, &position);
		} else {
			wakeup_add_zero(buffer, &position);
		}
	}

	wakeup_add_payload(&position, buffer, payload, payload_length);
	return position;
}

uint16_t broadcast_build_packet(uint8_t *buffer, uint8_t *payload,
		uint16_t payload_length) {

	/* build the wake-up message */
	uint16_t position = 0;
	uint8_t count;

	/* Carrier burst (for calibration). */
	wakeup_add_value(buffer, &position, LF_LOGIC_ONE, LENGTH_CB_CC);

	/* Separation bit */
	wakeup_add_value(buffer, &position, LF_LOGIC_ZERO, LENGTH_WU_BIT_CC);

	/* Preamble. */
	for (count = 0; count < LENGTH_PREAMBLE; count++) {
		wakeup_add_one(buffer, &position);
	}

	uint8_t * UARTpayload = malloc(2*payload_length);
	uint8_t i = 0;

	/* Convert payload to (LE)UART frame */
	UARTpayload[i++]   = 0xFE;
	UARTpayload[i++] = swapBits(payload[0]);
	UARTpayload[i++] = ((evenParityBit(&payload[0]) << 7) | 0x78 | ((swapBits(payload[1]) >> 6) & 0x03));
	UARTpayload[i++] = (swapBits(payload[1]) << 2) | (evenParityBit(&payload[1]) << 1) | 0x01;

	wakeup_add_payload(&position, buffer, UARTpayload, i);
	return position;
}

uint8_t swapBits(uint8_t data) {
	uint8_t swappedData = 0;
	uint8_t countBit;
	for (countBit = 0; countBit < 8; countBit++) {
		if ((data & (1 << (7 - countBit)))) {
			/* Write 1. */
			swappedData |= (1 << countBit);
		}
	}
	return swappedData;
}

uint8_t evenParityBit(uint8_t *data) {
	uint8_t parity = 0;
	uint8_t countBit;
	for (countBit = 0; countBit < 8; countBit++) {
		if ((*data & (1 << (7 - countBit)))) {
			parity++;
		}
	}

	return ((parity % 2) & 0x01);
}

uint8_t oddParityBit(uint8_t *data) {
	uint8_t parity = 0;
	uint8_t countBit;
	for (countBit = 0; countBit < 8; countBit++) {
		if ((*data & (1 << (7 - countBit)))) {
			parity++;
		}
	}

	return (((parity % 2) + 1) & 0x01);
}

void wakeup_add_payload(uint16_t *position, uint8_t *buffer, uint8_t *payload,
		uint16_t payload_length) {
	uint8_t countByte, countBit;
	for (countBit = 0; countBit < 8; countBit++) {
		if ((payload_length & (1 << (7 - countBit)))) {
			/* Write 1. */
			wakeup_add_one(buffer, position);
		} else {
			/* Write 0. */
			wakeup_add_zero(buffer, position);
		}
	}
	for (countByte = 0; countByte < payload_length; countByte++) {
		for (countBit = 0; countBit < 8; countBit++) {
			if ((payload[countByte] & (1 << (7 - countBit)))) {
				/* Write 1. */
				wakeup_add_one(buffer, position);
			} else {
				/* Write 0. */
				wakeup_add_zero(buffer, position);
			}
		}
	}
}


// Not valid for LF frequency different from 123 kHz
void wakeup_show_on_pin(uint8_t *buffer, uint16_t length,
		GPIO_Port_TypeDef port, uint8_t pin) {
	GPIO_PinModeSet(port, pin, gpioModePushPull, 0);
	volatile uint32_t * targetSetRegister = (volatile uint32_t *) (0x40006000
			+ port * 9 + 16);
	volatile uint32_t * targetClearRegister = (volatile uint32_t *) (0x40006000
			+ port * 9 + 20);
	uint32_t pinMask = 1 << pin;
	uint16_t i = 0, j = 0;

	/* Important: The loops below are necessary and result in a pretty
	 accurate timing of 123 kHz. */

	*targetClearRegister = pinMask;
	for (i = 0; i < 100; i++)
		;
	for (i = 0; i < length; i++) {
		*targetClearRegister = pinMask;
		for (j = 0; j < 6; j++)
			;
		if (buffer[i] == 0xFF) {
			*targetClearRegister = pinMask;
		} else {
			*targetSetRegister = pinMask;
		}
		for (j = 0; j < 7; j++)
			;
		// TODO (sesters): To get a more realistic output,
		// slow this loop down.
	}
	for (i = 0; i < 10; i++)
		;
	*targetClearRegister = pinMask;

}
