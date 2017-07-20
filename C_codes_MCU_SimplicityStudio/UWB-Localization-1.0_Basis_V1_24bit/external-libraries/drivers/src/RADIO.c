#include "RADIO.h"

#define AVAILABLE_BYTES_IN_RX_FIFO 40
#define AVAILABLE_BYTES_IN_TX_FIFO 60

#define MIN(a, b) ( ((a) > (b))?(b):(a) )

/* RF sending power (-15 dbm, -10 dbm, -6dbm, +0 dBm, +10 dBm, +12 dBm @ 868 MHz) */
const uint8_t pa_table[] = { 0x1D, 0x26, 0x37, 0x50, 0xC5, 0xC0 };

void radio_init(struct radio_transfer *xfer, TIMER_TypeDef *timer,
		CMU_Clock_TypeDef timer_clock, IRQn_Type timer_irq) {
	TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;

	xfer->timer = timer;

	/* Enable clock for the given timer module. */
	CMU_ClockEnable(timer_clock, true);

	timer_init.enable = false;
	timer_init.debugRun = true;
	timer_init.prescale = timerPrescale1024;
	timer_init.clkSel = timerClkSelHFPerClk;
	timer_init.fallAction = timerInputActionNone;
	timer_init.riseAction = timerInputActionNone;
	timer_init.mode = timerModeUp;
	timer_init.oneShot = true;

	TIMER_IntEnable(timer, TIMER_IF_OF);
	NVIC_EnableIRQ(timer_irq);

	/* Configure the given timer. */
	TIMER_Init(timer, &timer_init);
}

static void handle_transfer_rx_threshold(struct radio_transfer *xfer) {
	uint8_t n;

	if (!xfer->length_received) {
		/* TODO: Comment: Read first byte (length byte). */
		cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, &xfer->length, 1);

		if (!xfer->length) {
#if DEBUG
			printf("ERROR: Length byte is zero\n");
#endif
			cc1101_spi_strobe(CC1101_SIDLE);
			cc1101_spi_strobe(CC1101_SFRX);
		}

		/* TODO: Comment: Do not empty the FIFO due to RX FIFO bug (see CC1101 Errata). */
		/* TODO: Comment: Read 3 bytes less than the number of available bytes because it could be the appended status bytes if the packet is
		 AVAILABLE_BYTES_IN_RX_FIFO - 3 bytes long. */
		xfer->num_bytes_left = xfer->length - (AVAILABLE_BYTES_IN_RX_FIFO - 3);
		xfer->num_bytes = xfer->num_bytes + (AVAILABLE_BYTES_IN_RX_FIFO - 3);
		xfer->length_received = true;

		cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, xfer->buffer,
				AVAILABLE_BYTES_IN_RX_FIFO - 3);
#if DEBUG
		if (xfer->length < (AVAILABLE_BYTES_IN_RX_FIFO - 3)) {
			printf("ERROR BUFFER: %u %u\n", xfer->length, (AVAILABLE_BYTES_IN_RX_FIFO - 3));
			printf("0x%02ju\n", (intmax_t)GPIO_IntGet());

			uint8_t k;

			for (k = 0; k < (AVAILABLE_BYTES_IN_RX_FIFO - 3); k++) {
				printf("0x%02x ", xfer->buffer[k]);
			}
			printf("\n");

//	while (true);
		}
#endif

		return;
	}

	n = MIN(AVAILABLE_BYTES_IN_RX_FIFO - 1, xfer->num_bytes_left);
//printf("%u - %u\n", xfer->num_bytes, n);
	cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, xfer->buffer + xfer->num_bytes,
			n);
//	cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, xfer->buffer, n);

	xfer->num_bytes_left = xfer->num_bytes_left - n;
	xfer->num_bytes = xfer->num_bytes + n;
}

static void handle_transfer_tx_threshold(struct radio_transfer *xfer) {
	uint8_t n;

	if (xfer->num_bytes_left) {
		n = MIN(xfer->num_bytes_left, AVAILABLE_BYTES_IN_TX_FIFO);
		cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO,
				xfer->buffer + xfer->num_bytes, n);

		xfer->num_bytes_left -= n;
		xfer->num_bytes += n;
	}

	if (xfer->num_bytes_left == 0) { // already sent out every packet that fits into the fifo
		if (xfer->send_inf_wakeup_packet) {

			/*cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO, xfer->buffer,
			 xfer->num_last_bytes);*/
			cc1101_spi_write_reg(CC1101_REG_PKTCTRL0, 0x00); // change to fixed packet length
			GPIO_IntEnable((1 << CC1101_GDO0_PIN));
		}
	}
}

void radio_handle_transfer_threshold(struct radio_transfer *xfer) {
	if (xfer->receive_mode)
		handle_transfer_rx_threshold(xfer);
	else
		handle_transfer_tx_threshold(xfer);
}

void radio_handle_transfer_end(struct radio_transfer *xfer) {
	/* TODO: Comment: Disable both interrupts. */
	GPIO_IntDisable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

	if (xfer->receive_mode) {
		if (!xfer->length_received) {
			cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, &xfer->length, 1);
			xfer->length_received = true;
			xfer->num_bytes_left = xfer->length;
		}

		if (!xfer->length) {
#if DEBUG
			printf("ERROR2: Length byte is zero\n");
#endif
//while (true);
			cc1101_spi_strobe(CC1101_SIDLE);
			cc1101_spi_strobe(CC1101_SFRX);
		}

		if (xfer->num_bytes_left) {
			if (xfer->num_bytes + xfer->num_bytes_left > 255) {
#if DEBUG
				printf("ERROR BUFFER: %u %u\n", xfer->num_bytes, xfer->num_bytes_left);
				printf("Length: %u\n", xfer->length);
				printf("0x%02ju\n", (intmax_t)GPIO_IntGet());
#endif
				//while (true);
				cc1101_spi_strobe(CC1101_SIDLE);
				cc1101_spi_strobe(CC1101_SFRX);
			}
//printf("%u - %u\n", xfer->num_bytes, xfer->num_bytes_left);
			cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO,
					xfer->buffer + xfer->num_bytes, xfer->num_bytes_left);
//			cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, xfer->buffer, xfer->num_bytes_left);
		}

		cc1101_spi_read_burst_reg(CC1101_REG_RXFIFO, xfer->status, 2);
	}

	xfer->finished = true;
}

void radio_handle_timeout(struct radio_transfer *xfer) {
	xfer->timedout = true;
}

uint8_t wakeup_build_packet(uint8_t *buffer, const uint8_t *payload,
		uint8_t length) {
	/* TODO: Comment. */
	if (length > RADIO_MAX_PAYLOAD_LENGTH)
		/* TODO: Return error code? */
		length = 254;

	/* TODO: Comment: Use memmov instead of memcpy because buffer and payload might have the same source. */
	memmove(buffer + 1, payload, length);
	buffer[0] = length;

	return length + 1;
}

bool radio_send_packet_use_CSMA_CA(struct radio_transfer *xfer, uint8_t *buffer,
		uint8_t length) {
	bool TX_successful = false;

	/* TODO: Comment: Change the functionality of the GDO2-Pin and the threshold value. */
	cc1101_spi_write_reg(CC1101_REG_IOCFG2, 0x02);
	cc1101_spi_write_reg(CC1101_REG_FIFOTHR, 0x0e);

	/* Disable interrupt before re-configuring it. */
	GPIO_IntDisable(1 << CC1101_GDO2_PIN);

	/*
	 * Configure interrupt on falling edge for GDO2 and don't enable it -
	 * this will be done in the radio_send_packet function.
	 */
	GPIO_IntConfig(CC1101_GDO2_PORT, CC1101_GDO2_PIN, false, true, false);

	xfer->receive_mode = false;
	xfer->buffer = buffer;
	xfer->finished = false;

	/* TODO: Comment: Check if the data can be send all at once. */
	if (length > CC1101_FIFO_SIZE) {
		xfer->num_bytes_left = length - CC1101_FIFO_SIZE;
		xfer->num_bytes = CC1101_FIFO_SIZE;

		cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO, xfer->buffer,
				CC1101_FIFO_SIZE);
	} else {
		xfer->num_bytes_left = 0;
		xfer->num_bytes = length;

		cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO, xfer->buffer, length);
	}

	// Enter RX Mode
	cc1101_spi_strobe(CC1101_SRX);
	TIMER_start_us(TIMER2, 1000);
	// Wait for RX Mode to be entered
	while (cc1101_spi_read_reg(CC1101_REG_MARCSTATE) != CC1101_MARC_STATE_RX
			&& !TIMER_US_TIMEOUT)
		;
	if (TIMER_US_TIMEOUT) {
		cc1101_spi_strobe(CC1101_SIDLE);
		return false;
	}
	TIMER_stop_us(TIMER2 );
	// Wait for valid RSSI value
	TIMER_start_us(TIMER2, 550);
	while (!TIMER_US_TIMEOUT)
		;
	TIMER_stop_us(TIMER2 );
	// Try to enter TX Mode
	cc1101_spi_strobe(CC1101_STX);
	// Poll if radio enters TX state
	TIMER_start_us(TIMER2, 50);
	while (cc1101_spi_read_reg(CC1101_REG_MARCSTATE) != CC1101_MARC_STATE_TX
			&& !TIMER_US_TIMEOUT)
		;
	if (!TIMER_US_TIMEOUT) {
		TX_successful = true;

		//TODO: DEBUG:
		#if UART_DEBUG
		UART_WriteString("TX OK\r\n", sizeof("TX OK\r\n"));
		#endif

		/* TODO: Comment: Re-enable interrupts. */
		GPIO_IntEnable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));
		while (!xfer->finished)
			;
	}
	TIMER_stop_us(TIMER2 );
	cc1101_spi_strobe(CC1101_SIDLE);

	return TX_successful;
}

void radio_send_packet(struct radio_transfer *xfer, uint8_t *buffer,
		uint8_t length) {
	/* TODO: Comment: Change the functionality of the GDO2-Pin and the threshold value. */
	cc1101_spi_write_reg(CC1101_REG_IOCFG2, 0x02);
	cc1101_spi_write_reg(CC1101_REG_FIFOTHR, 0x0e);

	/* Disable interrupt before re-configuring it. */
	GPIO_IntDisable(1 << CC1101_GDO2_PIN);

	/*
	 * Configure interrupt on falling edge for GDO2 and don't enable it -
	 * this will be done in the radio_send_packet function.
	 */
	GPIO_IntConfig(CC1101_GDO2_PORT, CC1101_GDO2_PIN, false, true, false);

	xfer->receive_mode = false;
	xfer->buffer = buffer;
	xfer->finished = false;

	/* TODO: Comment: Check if the data can be send all at once. */
	if (length > CC1101_FIFO_SIZE) {
		xfer->num_bytes_left = length - CC1101_FIFO_SIZE;
		xfer->num_bytes = CC1101_FIFO_SIZE;

		cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO, xfer->buffer,
				CC1101_FIFO_SIZE);
	} else {
		xfer->num_bytes_left = 0;
		xfer->num_bytes = length;

		cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO, xfer->buffer, length);
	}

	cc1101_spi_strobe(CC1101_STX);

	/* TODO: Comment: Re-enable interrupts. */
	GPIO_IntEnable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

	while (!xfer->finished) {
	}
}

void radio_send_inf_wakeup_packet(struct radio_transfer *xfer, uint8_t *buffer,
		uint16_t length) {

	uint16_t fixed_packet_length;
	uint16_t bytes_left;
	uint8_t num_packets;

	uint8_t marcstate;

	xfer->receive_mode = false;
	xfer->buffer = buffer;
	xfer->finished = false;
	xfer->send_inf_wakeup_packet = true;

	bytes_left = length;
	fixed_packet_length = (bytes_left - CC1101_FIFO_SIZE)
			% AVAILABLE_BYTES_IN_TX_FIFO;
	xfer->num_last_bytes = fixed_packet_length;

	/* Set the number of bytes of the last packet because it will be sent
	 * with fixed packet length mode.
	 */
	cc1101_spi_write_reg(CC1101_REG_PKTLEN, fixed_packet_length);

	/* Disable interrupt before re-configuring it. */
	GPIO_IntDisable(1 << CC1101_GDO2_PIN);

	/*
	 * Configure interrupt on falling edge for GDO2 and don't enable it -
	 * this will be done in the radio_send_packet function.
	 */
	GPIO_IntConfig(CC1101_GDO2_PORT, CC1101_GDO2_PIN, false, true, false);

	/* Set infinite packet length mode. crc-check, whitening off */
	cc1101_spi_write_reg(CC1101_REG_PKTCTRL0, 0x02);
	/* Write the first CC1101_FIFO_SIZE (64) bytes. */
	cc1101_spi_write_burst_reg(CC1101_REG_TXFIFO, xfer->buffer,
			CC1101_FIFO_SIZE);

	bytes_left = bytes_left - CC1101_FIFO_SIZE;
	num_packets = bytes_left / AVAILABLE_BYTES_IN_TX_FIFO;

	xfer->num_bytes_left = bytes_left;
	xfer->num_bytes = CC1101_FIFO_SIZE;

	/* Enable interrupt pins to refill tx-buffer and detect sent package*/
	GPIO_IntEnable((1 << CC1101_GDO2_PIN)); // GPIO_IntEnable((1 << CC1101_GDO2_PIN | (1 << CC1101_GDO0_PIN)));

	//marcstate = cc1101_spi_read_reg(CC1101_REG_MARCSTATE);
	/* Enter transmission mode. */
	cc1101_spi_strobe(CC1101_STX);
	//marcstate = cc1101_spi_read_reg(CC1101_REG_MARCSTATE);
	/* Wait until the packet is sent. */
	while (!(xfer->finished)) {
		//marcstate = cc1101_spi_read_reg(CC1101_REG_MARCSTATE);
	}
	//marcstate = cc1101_spi_read_reg(CC1101_REG_MARCSTATE);
	/* clear send_inf_wakeup_packet variable */
	xfer->send_inf_wakeup_packet = false;

	//marcstate=cc1101_spi_read_reg(CC1101_REG_MARCSTATE);
	/* TODO: The cc1101 goes into error state after the fixed_packet length
	 mode. Only way to pass is set chip idle and flush tx fifo...
	 probably found the error...seems to was the last unsent bits in fixed
	 packets length (handle_tx_threshold())*/
	//cc1101_spi_strobe(CC1101_SIDLE);
	//cc1101_spi_strobe(CC1101_SFTX);
	//marcstate=cc1101_spi_read_reg(CC1101_REG_MARCSTATE);
}

int radio_receive_packet(struct radio_transfer *xfer, uint8_t *buffer,
		uint8_t *length, uint16_t timeout_ms) {
	uint32_t timeout;

	/* Disable interrupt before re-configuring it. */
	GPIO_IntDisable(1 << CC1101_GDO2_PIN);

	/*
	 * Configure interrupt on rising edge for GDO2 and don't enable it -
	 * this will be done in the radio_send_packet function.
	 */
	GPIO_IntConfig(CC1101_GDO2_PORT, CC1101_GDO2_PIN, true, false, false);

	/* TODO: Comment: Change the functionality of the GDO2-Pin and the threshold value. */
	cc1101_spi_write_reg(CC1101_REG_IOCFG2, 0x00);
	cc1101_spi_write_reg(CC1101_REG_FIFOTHR, 0x09);

	xfer->receive_mode = true;
	xfer->length_received = false;
	xfer->num_bytes = 0;
	xfer->finished = false;
	xfer->buffer = buffer;
	xfer->timedout = false;

	/* TODO: Magic number: 4760 = max timeout value. @28MHz*/
	/* TODO: Magic number: 3195 = max timeout value. @21MHz*/
	if (timeout_ms > 3195)
		timeout_ms = 3195;

	/* TODO: Magic number: 13768 = 1 second in counter value. @28MHz*/
	/* TODO: Magic number: 20508 = 1 second in counter value. @21MHz*/
	timeout = (20508 * timeout_ms) / 1000UL;

	if (timeout) {
		TIMER_CounterSet(xfer->timer, 0);
		TIMER_TopSet(xfer->timer, timeout);
		TIMER_Enable(xfer->timer, true);
	}

	cc1101_spi_strobe(CC1101_SRX);

	/* TODO: Comment: Re-enable interrupts. */
	GPIO_IntEnable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

	while (!xfer->finished && !xfer->timedout) {  //Yitong 0x1533720 <<
	}

	/* TODO: Comment: Disable and reset the timer. */
	TIMER_Enable(xfer->timer, false);
	TIMER_CounterSet(xfer->timer, 0);

	if (xfer->timedout) {
		/* TODO: Comment: Disable interrupts to prevent filling the fifo with incoming data. */
		GPIO_IntDisable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		cc1101_spi_strobe(CC1101_SIDLE);
		cc1101_spi_strobe(CC1101_SFRX);

		/* TODO: Comment: Clear possible pending interrupts. */
		GPIO_IntClear((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		return RADIO_ERR_TIMEOUT;
	}

	if ((xfer->status[1] & CC1101_LQI_CRC_OK_BM) != CC1101_LQI_CRC_OK_BM) {
		/* TODO: Comment: Disable interrupts to prevent filling the fifo with incoming data. */
		GPIO_IntDisable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		cc1101_spi_strobe(CC1101_SIDLE);
		cc1101_spi_strobe(CC1101_SFRX);

		/* TODO: Comment: Clear possible pending interrupts. */
		GPIO_IntClear((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		return RADIO_ERR_CRC;
	}

	*length = xfer->length;

	return RADIO_OK;
}

int radio_receive_packet_DWM1000(struct radio_transfer *xfer, uint8_t *buffer,
		uint16_t *length, uint16_t timeout_ms) {
	uint32_t timeout;

	/* Disable interrupt before re-configuring it. */
	GPIO_IntDisable(1 << CC1101_GDO2_PIN);

	/*
	 * Configure interrupt on rising edge for GDO2 and don't enable it -
	 * this will be done in the radio_send_packet function.
	 */
	GPIO_IntConfig(CC1101_GDO2_PORT, CC1101_GDO2_PIN, true, false, false);

	/* TODO: Comment: Change the functionality of the GDO2-Pin and the threshold value. */
	cc1101_spi_write_reg(CC1101_REG_IOCFG2, 0x00);
	cc1101_spi_write_reg(CC1101_REG_FIFOTHR, 0x09);

	xfer->receive_mode = true;
	xfer->length_received = false;
	xfer->num_bytes = 0;
	xfer->finished = false;
	xfer->buffer = buffer;
	xfer->timedout = false;

	/* TODO: Magic number: 4760 = max timeout value. @28MHz*/
	/* TODO: Magic number: 3195 = max timeout value. @21MHz*/
	if (timeout_ms > 3195)
		timeout_ms = 3195;

	/* TODO: Magic number: 13768 = 1 second in counter value. @28MHz*/
	/* TODO: Magic number: 20508 = 1 second in counter value. @21MHz*/
	timeout = (20508 * timeout_ms) / 1000UL;

	if (timeout) {
		TIMER_CounterSet(xfer->timer, 0);
		TIMER_TopSet(xfer->timer, timeout);
		TIMER_Enable(xfer->timer, true);
	}

	cc1101_spi_strobe(CC1101_SRX);

	/* TODO: Comment: Re-enable interrupts. */
	GPIO_IntEnable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

	while (!xfer->finished && !xfer->timedout) {
	}

	/* TODO: Comment: Disable and reset the timer. */
	TIMER_Enable(xfer->timer, false);
	TIMER_CounterSet(xfer->timer, 0);

	if (xfer->timedout) {
		/* TODO: Comment: Disable interrupts to prevent filling the fifo with incoming data. */
		GPIO_IntDisable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		cc1101_spi_strobe(CC1101_SIDLE);
		cc1101_spi_strobe(CC1101_SFRX);

		/* TODO: Comment: Clear possible pending interrupts. */
		GPIO_IntClear((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		return RADIO_ERR_TIMEOUT;
	}

	if ((xfer->status[1] & CC1101_LQI_CRC_OK_BM) != CC1101_LQI_CRC_OK_BM) {
		/* TODO: Comment: Disable interrupts to prevent filling the fifo with incoming data. */
		GPIO_IntDisable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		cc1101_spi_strobe(CC1101_SIDLE);
		cc1101_spi_strobe(CC1101_SFRX);

		/* TODO: Comment: Clear possible pending interrupts. */
		GPIO_IntClear((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

		return RADIO_ERR_CRC;
	}

	*length = xfer->length;

	return RADIO_OK;
}

void radio_receive_packet_polling_init(struct radio_transfer *xfer,
		uint8_t *buffer) {

	/* Disable interrupt before re-configuring it. */
	GPIO_IntDisable(1 << CC1101_GDO2_PIN);

	/*
	 * Configure interrupt on rising edge for GDO2 and don't enable it -
	 * this will be done in the radio_send_packet function.
	 */
	GPIO_IntConfig(CC1101_GDO2_PORT, CC1101_GDO2_PIN, true, false, false);

	/* TODO: Comment: Change the functionality of the GDO2-Pin and the threshold value. */
	cc1101_spi_write_reg(CC1101_REG_IOCFG2, 0x00);
	cc1101_spi_write_reg(CC1101_REG_FIFOTHR, 0x09);

	xfer->receive_mode = true;
	xfer->length_received = false;
	xfer->num_bytes = 0;
	xfer->finished = false;
	xfer->buffer = buffer;
	xfer->timedout = false;

	cc1101_spi_strobe(CC1101_SRX);

	/* TODO: Comment: Re-enable interrupts. */
	GPIO_IntEnable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));
}

int radio_receive_packet_polling(struct radio_transfer *xfer, uint8_t *length) {

	if (xfer->finished) {

		if ((xfer->status[1] & CC1101_LQI_CRC_OK_BM) != CC1101_LQI_CRC_OK_BM) {
			/* TODO: Comment: Disable interrupts to prevent filling the fifo with incoming data. */
			GPIO_IntDisable((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

			cc1101_spi_strobe(CC1101_SIDLE);
			cc1101_spi_strobe(CC1101_SFRX);

			/* TODO: Comment: Clear possible pending interrupts. */
			GPIO_IntClear((1 << CC1101_GDO0_PIN) | (1 << CC1101_GDO2_PIN));

			return RADIO_ERR_CRC;
		}

		*length = xfer->length;

		return RADIO_OK;
	}
	// Data not finished
	return 1;
}

bool radio_channel_busy(void) {
	// start with carrier sense
	GPIO_PinModeSet(CC1101_GDO0_PORT, CC1101_GDO0_PIN, gpioModeInput, true);
	cc1101_spi_write_reg(CC1101_REG_IOCFG0, CC1101_CARRIER_SENSE);
	//cc1101_spi_write_reg(CC1101_REG_AGCCTRL1, 0x09);
	// Enter read mode.
	cc1101_spi_strobe(CC1101_SRX);
	while (!CC1101_REG_MARCSTATE == CC1101_STATE_RX) {
	} // wait to enter receive mode
	radio_wait(500); // wait 500 us until the RSSI value is valid
	// TODO: Don't just wait. Here we could go into EM2 to safe energy and react either on the interrupt of the CS pin or the 500 us are over
	if (GPIO_PinInGet(CC1101_GDO0_PORT, CC1101_GDO0_PIN)) {
		cc1101_spi_strobe(CC1101_SIDLE);
		return true;
	} else { // channel is clear
		cc1101_spi_strobe(CC1101_SIDLE);
		return false;
	}

}

void radio_wait(uint16_t uDelay) {

	/* adjustment factor for 14MHz oscillator, based on the timing of this whole function with speed optimization on, could probably be done in a prettier way. */
	uint16_t cycle_delay = uDelay * 14 - 28;
	/* Enable clock for TIMER0 module */
	//CMU_ClockEnable(cmuClock_TIMER1, true);
	/* Configure TIMER */
	//TIMER_Init(TIMER1, &timerInit);
	/* Reset Timer */
	TIMER_CounterSet(TIMER1, 0);
	TIMER_IntDisable(TIMER1, TIMER_IF_OF);
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	/* Start TIMER0 */TIMER1 ->CMD = TIMER_CMD_START;
	/* Wait until counter value is over top */
	while (TIMER1 ->CNT < cycle_delay) {
		/* Do nothing, just wait */
	}
	TIMER1 ->CMD = TIMER_CMD_STOP;
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);
}

void cc1101_change_config_to(uint8_t configuration, uint8_t * patable) {
	cc1101_spi_powerup_reset();

	switch (configuration) {

	case CC1101_DATA_38kBaud_CONFIG:
		cc1101_spi_write_config(&cc1101_config_38kbaud);
		break;

	case CC1101_WAKEUP_CONFIG_NARROW_BAND:
		cc1101_spi_write_config(&cc1101_wakeup_config_narrow_band);
		break;

	case CC1101_SPECTRAL_ANALYSIS:
		cc1101_spi_write_config(&cc1101_spectral_analysis);
		break;

	default:
		// Set to WAKEUP_CONFIG

	case CC1101_WAKEUP_CONFIG:
		cc1101_spi_write_config(&cc1101_wakeup_config);
		cc1101_spi_write_reg(CC1101_REG_PKTCTRL0, 0x00);
		break;
	}

	//Set PA table:
	cc1101_spi_write_burst_reg(CC1101_REG_PATABLE, patable, 8);
	//cc1101_spi_write_burst_reg(CC1101_REG_PATABLE, pa_table + power, 1);

	GPIO_IntDisable(1 << CC1101_GDO0_PIN);
	GPIO_IntConfig(CC1101_GDO0_PORT, CC1101_GDO0_PIN, false, true, true);
}

void cc1101_power_down(void) {
	cc1101_spi_strobe(CC1101_SIDLE);
	cc1101_spi_strobe(CC1101_SPWD);
}

void cc1101_set_patable(uint8_t * patable) {

	// Set PA_TABLE by setting the pointer register FREND0[2:0] and writing the PA value
	cc1101_spi_write_burst_reg(CC1101_REG_PATABLE, patable, 8);

}

void cc1101_send_wakeup_packet_8bit_addr(struct radio_transfer *xfer,
		uint8_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t options, uint16_t measurements) {

	// Carrier Bugfix - Flush TX FIFO
	cc1101_spi_strobe(CC1101_SFTX);
	TIMER_delay_ms(TIMER0, 10, false);
	cc1101_spi_strobe(CC1101_SIDLE);

	uint8_t tmp_buffer[800] = { 0 };
	uint8_t payload[16] = { 0 };
	int16_t ret;

	uint16_t i = 0;
	// Add My Sender ID
	payload[i++] = (SOURCE_ID >> 16);  	// MY ID High Byte
	payload[i++] = (SOURCE_ID >> 8);  	// MY ID Middle Byte
	payload[i++] = SOURCE_ID & 0xFF;  	// MY ID Low Byte
	// Add number of measurements
	payload[i++] = measurements >> 8;	// Measurements High Byte
	payload[i++] = measurements & 0xFF; // Measurements Low Byte
	// Add option Byte
	payload[i++] = options;				// OPTION BYTE
	// Add CRC
	uint16_t chksum = CalcBlockCrc(payload, i);
	payload[i++] = chksum >> 8;			// CRC High Byte
	payload[i++] = chksum & 0xFF;		// CRC Low Byte

	ret = wakeup_build_packet_8bit(DESTINATION_ID, tmp_buffer, payload, i);

	/* Send Wake-up packet. */
	radio_send_inf_wakeup_packet(xfer, tmp_buffer, ret);
}

void cc1101_send_wakeup_packet_16bit_addr(struct radio_transfer *xfer,
		uint16_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t options, uint16_t measurements) {

	// Carrier Bugfix - Flush TX FIFO
	cc1101_spi_strobe(CC1101_SFTX);
	TIMER_delay_ms(TIMER0, 10, false);
	cc1101_spi_strobe(CC1101_SIDLE);

	uint8_t tmp_buffer[800] = { 0 };
	uint8_t payload[16] = { 0 };
	int16_t ret;

	uint16_t i = 0;
	// Add My Sender ID
	payload[i++] = (SOURCE_ID >> 16);  	// MY ID High Byte
	payload[i++] = (SOURCE_ID >> 8);  	// MY ID Middle Byte
	payload[i++] = SOURCE_ID & 0xFF;  	// MY ID Low Byte
	// Add number of measurements
	payload[i++] = measurements >> 8;	// Measurements High Byte
	payload[i++] = measurements & 0xFF; // Measurements Low Byte
	// Add option Byte
	payload[i++] = options;				// OPTION BYTE
	// Add CRC
	uint16_t chksum = CalcBlockCrc(payload, i);
	payload[i++] = chksum >> 8;			// CRC High Byte
	payload[i++] = chksum & 0xFF;		// CRC Low Byte

	ret = wakeup_build_packet_16bit(DESTINATION_ID, tmp_buffer, payload, i);

	/* Send Wake-up packet. */
	radio_send_inf_wakeup_packet(xfer, tmp_buffer, ret);
}

void cc1101_send_wakeup_packet_24bit_addr(struct radio_transfer *xfer,
		uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t options, uint16_t measurements) {

	// Carrier Bugfix - Flush TX FIFO
	cc1101_spi_strobe(CC1101_SFTX);
	TIMER_delay_ms(TIMER0, 10, false);
	cc1101_spi_strobe(CC1101_SIDLE);

	uint8_t tmp_buffer[800] = { 0 };
	uint8_t payload[16] = { 0 };
	int16_t ret;

	uint8_t UPPER_WU_BYTE = (uint8_t) (DESTINATION_ID >> 16);
	uint16_t address_16bit = (uint16_t) (DESTINATION_ID & 0xFFFF);

	uint16_t i = 0;
	// Add Upper Address Byte
	payload[i++] = UPPER_WU_BYTE;	// Additional 8bits of 16bit wakeup adress
	// Add My Sender ID
	payload[i++] = (SOURCE_ID >> 16);  	// MY ID High Byte
	payload[i++] = (SOURCE_ID >> 8);  	// MY ID Middle Byte
	payload[i++] = SOURCE_ID & 0xFF;  	// MY ID Low Byte
	// Add number of measurements
	payload[i++] = measurements >> 8;	// Measurements High Byte
	payload[i++] = measurements & 0xFF; // Measurements Low Byte
	// Add option Byte
	payload[i++] = options;				// OPTION BYTE
	// Add CRC
	uint16_t chksum = CalcBlockCrc(payload, i);
	payload[i++] = chksum >> 8;			// CRC High Byte
	payload[i++] = chksum & 0xFF;		// CRC Low Byte
	// Add Dummy Byte
	payload[i++] = 0xAA;  		// Caution: Do not use last dummy byte for data!

	ret = wakeup_build_packet_16bit(address_16bit, tmp_buffer, payload, i);

	/* Send Wake-up packet. */
	radio_send_inf_wakeup_packet(xfer, tmp_buffer, ret);
}

void cc1101_send_broadcast_packet(struct radio_transfer *xfer,
		uint8_t broadcastData) {

	// Carrier Bugfix - Flush TX FIFO
	cc1101_spi_strobe(CC1101_SFTX);
	TIMER_delay_ms(TIMER0, 10, false);
	cc1101_spi_strobe(CC1101_SIDLE);

	uint8_t tmp_buffer[205] = { 0 };
	uint8_t payload[16] = { 0 };
	int16_t ret;

	uint16_t i = 0;

	// Broadcast data is limited to one byte per packet!!
	payload[i++] = broadcastData;
	payload[i++] = '\r';

	ret = broadcast_build_packet(tmp_buffer, payload, i);

	/* Send Wake-up packet. */
	radio_send_inf_wakeup_packet(xfer, tmp_buffer, ret);
}

void cc1101_ack_reception(struct radio_transfer *xfer, uint32_t *SOURCE_ID,
		uint32_t *DESTINATION_ID, const uint8_t *key, const uint8_t *iv) {

	uint8_t length = 0;
	uint8_t ACK_data[10] = { 0 };
	uint8_t ACK_send[256] = { 0 };

	// Add Data
	// No Data for ACK

	// Build Header
	RFPacket_build_header(ACK_send, ACK_data, SOURCE_ID, DESTINATION_ID,
			CC_OPT_BYTE_ACK, &length);

	// Add length byte for CC
	RFPacket_add_cc_length(ACK_send, &length);

	// Encrypt send packet (AES128CBC)
	RFPacket_encrypt_AES128CBC(ACK_send, &length, key, iv);

#if USE_CSMA_CA
	// Pure CCA, wait for clear channel (max. 20ms), send if channel clear
	TIMER_start(TIMER0, 20); // TODO: Timeout length?
	bool TX_OK = radio_send_packet_use_CSMA_CA(xfer, ACK_send, length);
	while (!TX_OK && !TIMER_TIMEOUT) {
		TX_OK = radio_send_packet_use_CSMA_CA(xfer, ACK_send, length);
	}
	TIMER_stop(TIMER0 );
#else
	radio_send_packet(xfer, ACK_send, length);
#endif
}

bool cc1101_check_ack(struct radio_transfer *xfer, uint32_t *SOURCE_ID,
		uint32_t *DESTINATION_ID, const uint8_t *key, const uint8_t *iv,
		uint16_t timeout_ms) {

	uint8_t CC1101_receive[256] = { 0 };
	uint8_t len = 0;

	// Listen for ACK (with timeout)
	radio_receive_packet(xfer, CC1101_receive, &len, timeout_ms);

	// Decrypt received packet (AES128CBC)
	RFPacket_decrypt_AES128CBC(CC1101_receive, key, iv);

	// Check packet and remove header
	uint8_t MSG_TYPE = CC_OPT_BYTE_ACK;
	uint8_t ack = RFPacket_check_and_remove_header(CC1101_receive, SOURCE_ID,
			DESTINATION_ID, &MSG_TYPE);

	// Check if ACK
	if (ack == 0x06) {
		return true;  // ACK
	} else {
		return false;  // NOT ACK
	}
}

int8_t cc1101_read_rssi(void) {
	uint8_t rssi_dec;
	int16_t rssi_dBm;
	uint8_t rssi_offset = 74;
	rssi_dec = cc1101_spi_read_reg(CC1101_REG_RSSI);
	if (rssi_dec >= 128) {
		rssi_dBm = (int16_t) ((int16_t) (rssi_dec - 256) / 2) - rssi_offset;
	} else {
		rssi_dBm = (rssi_dec / 2) - rssi_offset;
	}
	if (rssi_dBm < -128) {
		rssi_dBm = -128;
	}
	return rssi_dBm;
}
