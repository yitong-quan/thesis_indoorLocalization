#include "AS3933.h"
#include "ANTENNA.h"
#include "config.h"

#ifdef AS3933_USART_CLK

volatile uint8_t UPPER_ADR_BYTE = 0x00;  // used for 24bit WU
uint8_t received_payload[16];

void AS3933_spi_init(uint32_t SPI_BAUDRATE) {

	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(AS3933_USART_CLK, true);

	GPIO_PinModeSet(AS3933_MOSI_PORT, AS3933_MOSI_PIN, gpioModePushPull, false);
	GPIO_PinModeSet(AS3933_MISO_PORT, AS3933_MISO_PIN, gpioModeInput, false);
	GPIO_PinModeSet(AS3933_CLK_PORT, AS3933_CLK_PIN, gpioModePushPull, false);

	/* Configure the chip select pin as output and clear it to ensure that
	 the chip is not active. */
	GPIO_PinModeSet(AS3933_CS_PORT, AS3933_CS_PIN, gpioModePushPull, false);

	init.baudrate = SPI_BAUDRATE;
	init.databits = usartDatabits8;
	init.msbf = true;
	init.master = true;
	/* CLKPOL = 0, CLKPHA = 1 */
	init.clockMode = usartClockMode1;

	USART_InitSync(AS3933_USART, &init);

	/* Configure USART location. */AS3933_USART ->ROUTE = AS3933_USART_LOC
			| (AS3933_USART ->ROUTE & ~_USART_ROUTE_LOCATION_MASK);

	/* Enable USART signals TX, RX and CLK. */
	AS3933_USART ->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN
			| USART_ROUTE_CLKPEN;
}

void AS3933_spi_write_reg(uint8_t address, uint8_t data) {
	GPIO_PinOutSet(AS3933_CS_PORT, AS3933_CS_PIN);

	USART_SpiTransfer(AS3933_USART, address | AS3933_WRITE_REG);
	USART_SpiTransfer(AS3933_USART, data);

	GPIO_PinOutClear(AS3933_CS_PORT, AS3933_CS_PIN);
}

void AS3933_spi_write_burst_reg(uint8_t address, const uint8_t *buffer,
		uint8_t count) {
	GPIO_PinOutSet(AS3933_CS_PORT, AS3933_CS_PIN);

	USART_SpiTransfer(AS3933_USART, address | AS3933_WRITE_REG);

	uint8_t i;
	for (i = 0; i < count; i++) {
		USART_SpiTransfer(AS3933_USART, buffer[i]);
	}

	GPIO_PinOutClear(AS3933_CS_PORT, AS3933_CS_PIN);
}

uint8_t AS3933_spi_read_reg(uint8_t address) {
	GPIO_PinOutSet(AS3933_CS_PORT, AS3933_CS_PIN);

	USART_SpiTransfer(AS3933_USART, address | AS3933_READ_REG);
	uint8_t data = USART_SpiTransfer(AS3933_USART, 0x00);

	GPIO_PinOutClear(AS3933_CS_PORT, AS3933_CS_PIN);

	return data;
}

void AS3933_spi_direct_cmd(uint8_t cmd) {
	GPIO_PinOutSet(AS3933_CS_PORT, AS3933_CS_PIN);

	USART_SpiTransfer(AS3933_USART, cmd | AS3933_DIRECT_CMD);
	USART_SpiTransfer(AS3933_USART, 0);

	GPIO_PinOutClear(AS3933_CS_PORT, AS3933_CS_PIN);
}

bool AS3933_calibrate(uint32_t SPI_BAUDRATE) {
	AS3933_spi_init(SPI_BAUDRATE);

	AS3933_USART ->CMD = USART_CMD_RXBLOCKEN;
	GPIO_PinOutSet(AS3933_CS_PORT, AS3933_CS_PIN);

	USART_Tx(AS3933_USART, 0xc2); // command trim_osc

	/* Wait until address and data are transmitted. */
	while (!(AS3933_USART ->STATUS & USART_STATUS_TXC))
		;

	AS3933_USART ->CMD = USART_CMD_RXBLOCKDIS;

	/* Calibrate the RC oscillator
	 * change spi clock to generate specified signal on sclk strobe,
	 * see AS3933 datasheet, p. 49
	 * fRC = fcarr/4 		@ fcarr = 125 kHz
	 * fRC = fcarr * 14 / 8 @ fcarr = 19.2 kHz
	 * */
#if		LF_FREQ == 125000
	USART_BaudrateSyncSet(AS3933_USART, 0, 31250);
#elif	LF_FREQ == 19200
	USART_BaudrateSyncSet(AS3933_USART, 0, 33600);
#endif

	/* transmit zeros until 65 clock periods have passed */
	uint8_t i;
	for (i = 0; i < 9; i++) {
		USART_Tx(AS3933_USART, 0x00);
	}

	/* Wait until address and data are transmitted. */
	while (!(AS3933_USART ->STATUS & USART_STATUS_TXC))
		;

	GPIO_PinOutClear(AS3933_CS_PORT, AS3933_CS_PIN);
	AS3933_USART ->CMD = USART_CMD_RXBLOCKDIS;

	/* change back to old configuration */
	AS3933_spi_init(SPI_BAUDRATE);

	// RC_CAL_OK Bit
	uint8_t REG14 = AS3933_spi_read_reg(AS3933_REG14);
	if (REG14 & AS3933_RC_CAL_OK) {
		return true;
	} else if (REG14 & AS3933_RC_CAL_KO) {
		return false;
	} else {
		return false;
	}
}

uint16_t convert_hex_to_manchester(uint8_t hex) {
	int8_t i = 8;
	uint16_t ret = 0;
	while (i > 0) {
		i--;
		ret <<= 2;
		if (hex & (1 << i)) {
			ret |= 2;
		} else {
			ret |= 1;
		}
	}
	return ret;
}

void AS3933_wakeup_pattern_8bit(uint8_t wakeup_node_id) {
	uint16_t address = convert_hex_to_manchester(wakeup_node_id);

	// First part of the address goes into 0x6, second part into 0x5
	AS3933_spi_write_reg(AS3933_REG6, (uint8_t) (address >> 8));
	AS3933_spi_write_reg(AS3933_REG5, (uint8_t) (address & 0xFF));
}

void AS3933_wakeup_pattern_16bit(uint16_t wakeup_node_id) {
	// First part of the address goes into 0x6, second part into 0x5
	AS3933_spi_write_reg(AS3933_REG6, (uint8_t) (wakeup_node_id >> 8));
	AS3933_spi_write_reg(AS3933_REG5, (uint8_t) (wakeup_node_id & 0xFF));
}

void AS3933_setup(bool manchester, bool ADDRESS_16BIT) {
	/* Reset AS3933 */
	AS3933_spi_direct_cmd(AS3933_CMD_PRESET_DEFAULT);

	if (ADDRESS_16BIT) {
		/* Activate coil 1, DAT_MASK on, enable 16bit Address (32bit Manchester) */
		AS3933_spi_write_reg(AS3933_REG0,
				AS3933_EN_A1 /*| AS3933_DAT_MASK */ | AS3933_PATT32);
	} else {
		/* Activate coil 1, DAT_MASK on */
		AS3933_spi_write_reg(AS3933_REG0, AS3933_EN_A1 /*| AS3933_DAT_MASK*/);
	}

#if		LF_FREQ == 125000
	#if MODE == TEST_WAKE_UP_SENSITIVITY
	/* Correlation off. AGC Up/Down on */
	AS3933_spi_write_reg(AS3933_REG1,
			 AS3933_EN_MANCH | AS3933_AGC_UD | AS3933_AGC_TLIM);
	#else
	/* Correlation on. AGC Up/Down on */
	AS3933_spi_write_reg(AS3933_REG1,
			AS3933_EN_WPAT | AS3933_EN_MANCH | AS3933_AGC_UD | AS3933_AGC_TLIM);
	#endif

	/* Frequency Tolerance setting relaxed, GainBoost 3dB on */
	AS3933_spi_write_reg(AS3933_REG2, AS3933_G_BOOST | AS3933_S_WU1_REL);

	/* Preamble > 0.8 ms, 4096 symbols/s */
	AS3933_spi_write_reg(AS3933_REG3, AS3933_FS_SCL_080 | AS3933_FS_ENV_4096);

	// No Gain Reduction, antenna shunt 3kohm, ON OFF 1ms
	AS3933_spi_write_reg(AS3933_REG4,
			AS3933_GR_0 | AS3933_D_RES_3k | AS3933_T_OFF_1MS);

	/* 8192 bit/s (4096 manchester symbols/s) datarate, no timeout */
	AS3933_spi_write_reg(AS3933_REG7, AS3933_T_HBIT_8192 | AS3933_T_OUT_0);

	// Band selection 95-150kHz, no artificial wakeup
	AS3933_spi_write_reg(AS3933_REG8, AS3933_BAND_95_150_kHz | AS3933_T_AUTO_0);
#elif	LF_FREQ == 19200


	#if MODE == TEST_WAKE_UP_SENSITIVITY
	/* Correlation off. AGC Up/Down on */
	AS3933_spi_write_reg(AS3933_REG1, AS3933_EN_MANCH | AS3933_AGC_UD | AS3933_AGC_TLIM);
	#else
	/* Correlation on. AGC Up/Down on */
	AS3933_spi_write_reg(AS3933_REG1,
				AS3933_EN_WPAT | AS3933_EN_MANCH | AS3933_AGC_UD | AS3933_AGC_TLIM);
	#endif

	/* Frequency Tolerance setting relaxed, GainBoost 3dB on */
	AS3933_spi_write_reg(AS3933_REG2, AS3933_G_BOOST | AS3933_S_WU1_REL);

	/* smallest slow envelope time constant (required preamble longer than 0.8 ms), fast envelope suitable for up to 4680 manchester symbols/s  */
	AS3933_spi_write_reg(AS3933_REG3, AS3933_FS_SCL_080 | AS3933_FS_ENV_4096);

	// No Gain Reduction, antenna shunt 3kohm, ON OFF 1ms
	AS3933_spi_write_reg(AS3933_REG4,
			AS3933_GR_0 | AS3933_D_RES_3k | AS3933_T_OFF_1MS);

	/* bit duration: 14 RC oscillator periods (equivalent to 4680 bit/s setup on AS3932),
	 * effective bit duration is 14*1/(14/8*fcarr) => 2400 bit/s => 1200 manchester symbols/s;
	 * no timeout is set */
	AS3933_spi_write_reg(AS3933_REG7, AS3933_T_HBIT_2340 | AS3933_T_OUT_0);

	/* OLD: bit duration: 7 RC oscillator periods (equivalent to 4680 bit/s setup on AS3932),
	 * effective bit duration is 7*1/(14/8*fcarr) => 4800 bit/s => 2400 manchester symbols/s;
	 * no timeout is set
	AS3933_spi_write_reg(AS3933_REG7, AS3933_T_HBIT_4680 | AS3933_T_OUT_0); */

	// Band selection 15-23kHz, no artificial wakeup
	AS3933_spi_write_reg(AS3933_REG8, AS3933_BAND_15_23_kHz | AS3933_T_AUTO_0);
#endif

}

// prepare wakeup interrupt pin
void AS3933_wakeup_init(bool config_interrupt) {
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(AS3933_WAKE_PORT, AS3933_WAKE_PIN, gpioModeInput, false);

	if (config_interrupt) {
		NVIC_EnableIRQ(GPIO_ODD_IRQn);

		/* Disable the interrupt for the WakeUp pin before configuring
		 it (see documentation of GPIO_IntConfig). */
		AS3933_wakeup_int_disable();
		GPIO_IntConfig(AS3933_WAKE_PORT, AS3933_WAKE_PIN, true, false, false);
	}
}

void AS3933_wakeup_int_enable(void) {
	GPIO_IntEnable(1 << AS3933_WAKE_PIN);
}

void AS3933_wakeup_int_disable(void) {
	GPIO_IntDisable(1 << AS3933_WAKE_PIN);
}

void AS3933_prepare_wakeup(uint32_t SPI_BAUDRATE, bool USE_16bit) {
	/* Prepare all pointers, arrays, etc. for the data-reception after wakeup. */
	GPIO_PinModeSet(AS3933_DCLK_PORT, AS3933_DCLK_PIN, gpioModeInput, 0);
	GPIO_PinModeSet(AS3933_DATA_PORT, AS3933_DATA_PIN, gpioModeInput, 0);

	/* Switch to AS3933 SPI Mode and (Re-)Start the AS3933 */
	AS3933_spi_init(SPI_BAUDRATE);
	AS3933_setup(true, USE_16bit);

	/* Start internal RC-Oscillator calibration */
	AS3933_calibrate(SPI_BAUDRATE);

	memset(received_payload, 0x00, sizeof(received_payload)); // AS3933 Received Data

	WAKEUP_active = true;
	WAKEUP_received = false;
	WAKEUP_24BIT = false;
	UPPER_ADR_BYTE = 0x00;
}

void AS3933_start_wakeup(void) {
	// Reset RSSI value
	AS3933_spi_direct_cmd(AS3933_CMD_RESET_RSSI);

	// Clear Wake Signal (go to listening mode)
	AS3933_spi_direct_cmd(AS3933_CMD_CLEAR_WAKE);

	/* Connect AS3933 with the antenna, start the interrupt. */
	AS3933_wakeup_init(true);
	AS3933_wakeup_int_enable();
}

void AS3933_enable_wakeup_by_8bit_address(uint8_t address,
		uint32_t SPI_BAUDRATE) {

	AS3933_prepare_wakeup(SPI_BAUDRATE, false);
	AS3933_wakeup_pattern_8bit(address);
	AS3933_start_wakeup();

}

void AS3933_enable_wakeup_by_16bit_address(uint16_t address,
		uint32_t SPI_BAUDRATE) {
	AS3933_prepare_wakeup(SPI_BAUDRATE, true);
	AS3933_wakeup_pattern_16bit(address);
	AS3933_start_wakeup();
}

void AS3933_enable_wakeup_by_24bit_address(uint32_t address,
		uint32_t SPI_BAUDRATE) {

	AS3933_prepare_wakeup(SPI_BAUDRATE, true);
	WAKEUP_24BIT = true;
	UPPER_ADR_BYTE = (uint8_t) (address >> 16);
	uint16_t address_16bit = (uint16_t) (address & 0x0000FFFF);
	AS3933_wakeup_pattern_16bit(address_16bit);
	AS3933_start_wakeup();
}

void AS3933_enable_wakeup_by_frequency_detection(uint32_t SPI_BAUDRATE) {

	AS3933_prepare_wakeup(SPI_BAUDRATE, false);
	AS3933_wakeup_pattern_8bit(0x00);
	AS3933_start_wakeup();
}

bool AS3933_receive_data(uint32_t *woke_up_by_ID, uint8_t * options, uint16_t *n_measurements, uint32_t SPI_BAUDRATE) {
	uint16_t numByte = 0;
	uint16_t numBit = 0;
	uint8_t length = 1;

	while (numByte <= length) {
		// Wait for LoHi-transition
		TIMER_start(TIMER0, 2);
		while (!(GPIO_PinInGet(AS3933_DCLK_PORT, AS3933_DCLK_PIN)) && !TIMER_TIMEOUT)
			;
		TIMER_stop(TIMER0);
		received_payload[numByte] <<= 1;

		if (GPIO_PinInGet(AS3933_DATA_PORT, AS3933_DATA_PIN)) {
			received_payload[numByte] |= 1;
		}

		if (numBit == 7) {
			numBit = 0;
			numByte++;
			if (numByte == 1) {
				length = received_payload[0];
			}
		} else {
			numBit++;
		}

		// Wait for HiLo-transition
		TIMER_start(TIMER0, 2);
		while (GPIO_PinInGet(AS3933_DCLK_PORT, AS3933_DCLK_PIN) && !TIMER_TIMEOUT)
			;
		TIMER_stop(TIMER0);
	}

	// Copy Buffer to compute payload
	uint16_t chksum_computed = 0;
	uint16_t chksum_received = 0;
	uint8_t i;
	length = sizeof(received_payload);
	//uint8_t tmp[length] = { 0 };
	uint8_t tmp[16] = { 0 };
	for (i = 0; i < length-1; i++) {
		tmp[i] = received_payload[i+1];
	}

	// Received data is in received_payload[], with received_payload[0] being the length of the payload
	// (so received_payload.length is received_payload[0] + 1)
	if (WAKEUP_24BIT)
	{
		// Check CRC
		chksum_computed = CalcBlockCrc(tmp, 7);
		chksum_received = (((uint16_t) received_payload[8]) << 8 | received_payload[9]);
		if (chksum_computed != chksum_received) {
			return false;
		}
		// Fetch Data
		*woke_up_by_ID = (((uint32_t) received_payload[2]) << 16 | ((uint32_t) received_payload[3]) << 8 | received_payload[4]);
		*n_measurements = ((uint16_t) received_payload[5] << 8 | received_payload[6]);
		*options = received_payload[7];
		// Check if Upper Address Byte is correct
		if (received_payload[1] == UPPER_ADR_BYTE) {
			RTC_delay_ms(5);
			return true;  // It's my wakeup address
		}
		return false;  // It's not my wakeup address
	}
	else
	{
		chksum_computed = CalcBlockCrc(tmp, 6);
		chksum_received = (((uint16_t) received_payload[7]) << 8 | received_payload[8]);
		if (chksum_computed != chksum_received)
		{
			return false;
		}
		*woke_up_by_ID = (((uint32_t) received_payload[1]) << 16 | ((uint32_t) received_payload[2]) << 8 | received_payload[3]);
		*n_measurements = ((uint16_t) received_payload[4] << 8 | received_payload[5]);
		*options = received_payload[6];

		return true;  // Everything's fine.
	}

}

void AS3933_EFM_sleep_enable_wake_up (uint32_t WAKEUP_ID)
{
	// Enable AS3933 WakeUp by Address
	#if		WU_ADDR_LENGTH == USE_8_BIT
		AS3933_enable_wakeup_by_8bit_address(WAKEUP_ID, AS3933_SPI_BAUDRATE);
	#elif	WU_ADDR_LENGTH == USE_16_BIT
		AS3933_enable_wakeup_by_16bit_address(WAKEUP_ID, AS3933_SPI_BAUDRATE);
	#elif	WU_ADDR_LENGTH == USE_24_BIT
		AS3933_enable_wakeup_by_24bit_address(WAKEUP_ID, AS3933_SPI_BAUDRATE);
	#endif

	// Switch antenna to WakeUp Receiver
	ANTENNA_switch(ANTENNA_DEST_WAKEUPRF);

	// Now enter SleepMode!!
	while (!WAKEUP_received)
	{
		__disable_irq();
		//EMU_EnterEM1();
		EMU_EnterEM2(true);  // change from EM3 to EM2  //always on //EMU_EnterEM1(); //
		__enable_irq();

	}
}

#endif /* def. AS3933_USART_CLK */
