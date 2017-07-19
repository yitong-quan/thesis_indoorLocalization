#include "em_gpio.h"
#include "CC1101.h"
#include "em_usart.h"
#include "em_cmu.h"

/* Definitions for register access. */
#define CC1101_WRITE_BURST	0x40
#define CC1101_READ_SINGLE	0x80
#define CC1101_READ_BURST	0xC0

//
void cc1101_spi_init(uint32_t SPI_BAUDRATE) {

	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(CC1101_MOSI_PORT, CC1101_MOSI_PIN, gpioModePushPull, false);
	GPIO_PinModeSet(CC1101_MISO_PORT, CC1101_MISO_PIN, gpioModeInput, false);
	GPIO_PinModeSet(CC1101_CLK_PORT, CC1101_CLK_PIN, gpioModePushPull, false);

	/*
	 * Configure the chip select pin as output and set it to ensure that the
	 * chip is not active.
	 */
	GPIO_PinModeSet(CC1101_CS_PORT, CC1101_CS_PIN, gpioModePushPull, true);
	CMU_ClockEnable(cmuClock_USART0, true);

	init.baudrate = SPI_BAUDRATE;
	init.databits = usartDatabits8;
	init.msbf = true;
	init.master = true;
	/* CLKPOL = 0, CLKPHA = 0 */
	init.clockMode = usartClockMode0;
	//	init.prsRxEnable = false;
	//	init.autoTx = false;

	USART_InitSync(USART0, &init);

	/* Configure USART location. */USART0 ->ROUTE = USART_ROUTE_LOCATION_LOC0
			| (USART0 ->ROUTE & ~_USART_ROUTE_LOCATION_MASK);

	/* Enable USART signals TX, RX and CLK. */
	USART0 ->ROUTE |=
			USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
}

//
void cc1101_spi_powerup_reset(void) {
#if 0
	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

	startup_delay(30);

	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	startup_delay(30);

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

	startup_delay(45);

	USART0->CMD = USART_CMD_RXBLOCKEN;
	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN));

	USART_Tx(USART0, CC1101_SRES);

	/* Wait until address and data are transmitted. */
	while (!(USART0->STATUS & USART_STATUS_TXC));

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN));

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);
	USART0->CMD = USART_CMD_RXBLOCKDIS;
#else
	cc1101_spi_strobe(CC1101_SRES);
#endif
}

//
void cc1101_spi_write_reg(uint8_t address, uint8_t data) {
	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN))
		;

	USART_SpiTransfer(USART0, address);
	USART_SpiTransfer(USART0, data);

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

}

//
void cc1101_spi_write_burst_reg(uint8_t address, const uint8_t * const buffer,
		uint8_t count) {
	uint8_t i;

	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN))
		;

	USART_SpiTransfer(USART0, address | CC1101_WRITE_BURST);

	for (i = 0; i < count; i++)
		USART_SpiTransfer(USART0, buffer[i]);

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

}

//
uint8_t cc1101_spi_read_reg(uint8_t address) {
	uint8_t data;

	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN))
		;

	if (address > 0x2f)
		address |= CC1101_READ_BURST;
	else
		address |= CC1101_READ_SINGLE;

	USART_SpiTransfer(USART0, address);
	data = USART_SpiTransfer(USART0, 0x00);

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

	return data;
}

//
void cc1101_spi_read_burst_reg(uint8_t address, uint8_t * const buffer,
		uint8_t count) {
	uint8_t i;

	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN))
		;

	USART_SpiTransfer(USART0, address | CC1101_READ_BURST);

	for (i = 0; i < count; i++)
		buffer[i] = USART_SpiTransfer(USART0, 0x00);

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

}

//
uint8_t cc1101_spi_strobe(uint8_t cmd) {
	uint8_t data;

	GPIO_PinOutClear(CC1101_CS_PORT, CC1101_CS_PIN);

	while (GPIO_PinInGet(CC1101_MISO_PORT, CC1101_MISO_PIN))
		;

	data = USART_SpiTransfer(USART0, cmd);

	GPIO_PinOutSet(CC1101_CS_PORT, CC1101_CS_PIN);

	return data;
}

//
void cc1101_spi_write_config(const struct cc1101_config *config) {
	cc1101_spi_write_reg(CC1101_REG_IOCFG2, config->iocfg2);
	cc1101_spi_write_reg(CC1101_REG_IOCFG1, config->iocfg1);
	cc1101_spi_write_reg(CC1101_REG_IOCFG0, config->iocfg0);
	cc1101_spi_write_reg(CC1101_REG_FIFOTHR, config->fifothr);
	cc1101_spi_write_reg(CC1101_REG_SYNC1, config->sync1);
	cc1101_spi_write_reg(CC1101_REG_SYNC0, config->sync0);
	cc1101_spi_write_reg(CC1101_REG_PKTLEN, config->pktlen);
	cc1101_spi_write_reg(CC1101_REG_PKTCTRL1, config->pktctrl1);
	cc1101_spi_write_reg(CC1101_REG_PKTCTRL0, config->pktctrl0);
	cc1101_spi_write_reg(CC1101_REG_ADDR, config->addr);
	cc1101_spi_write_reg(CC1101_REG_CHANNR, config->channr);
	cc1101_spi_write_reg(CC1101_REG_FSCTRL1, config->fsctrl1);
	cc1101_spi_write_reg(CC1101_REG_FSCTRL0, config->fsctrl0);
	cc1101_spi_write_reg(CC1101_REG_FREQ2, config->freq2);
	cc1101_spi_write_reg(CC1101_REG_FREQ1, config->freq1);
	cc1101_spi_write_reg(CC1101_REG_FREQ0, config->freq0);
	cc1101_spi_write_reg(CC1101_REG_MDMCFG4, config->mdmcfg4);
	cc1101_spi_write_reg(CC1101_REG_MDMCFG3, config->mdmcfg3);
	cc1101_spi_write_reg(CC1101_REG_MDMCFG2, config->mdmcfg2);
	cc1101_spi_write_reg(CC1101_REG_MDMCFG1, config->mdmcfg1);
	cc1101_spi_write_reg(CC1101_REG_MDMCFG0, config->mdmcfg0);
	cc1101_spi_write_reg(CC1101_REG_DEVIATN, config->deviatn);
	cc1101_spi_write_reg(CC1101_REG_MCSM2, config->mcsm2);
	cc1101_spi_write_reg(CC1101_REG_MCSM1, config->mcsm1);
	cc1101_spi_write_reg(CC1101_REG_MCSM0, config->mcsm0);
	cc1101_spi_write_reg(CC1101_REG_FOCCFG, config->foccfg);
	cc1101_spi_write_reg(CC1101_REG_BSCFG, config->bscfg);
	cc1101_spi_write_reg(CC1101_REG_AGCCTRL2, config->agcctrl2);
	cc1101_spi_write_reg(CC1101_REG_AGCCTRL1, config->agcctrl1);
	cc1101_spi_write_reg(CC1101_REG_AGCCTRL0, config->agcctrl0);
	cc1101_spi_write_reg(CC1101_REG_WOREVT1, config->worevt1);
	cc1101_spi_write_reg(CC1101_REG_WOREVT0, config->worevt0);
	cc1101_spi_write_reg(CC1101_REG_WORCTRL, config->worctrl);
	cc1101_spi_write_reg(CC1101_REG_FREND1, config->frend1);
	cc1101_spi_write_reg(CC1101_REG_FREND0, config->frend0);
	cc1101_spi_write_reg(CC1101_REG_FSCAL3, config->fscal3);
	cc1101_spi_write_reg(CC1101_REG_FSCAL2, config->fscal2);
	cc1101_spi_write_reg(CC1101_REG_FSCAL1, config->fscal1);
	cc1101_spi_write_reg(CC1101_REG_FSCAL0, config->fscal0);
	cc1101_spi_write_reg(CC1101_REG_RCCTRL1, config->rcctrl1);
	cc1101_spi_write_reg(CC1101_REG_RCCTRL0, config->rcctrl0);
	cc1101_spi_write_reg(CC1101_REG_FSTEST, config->fstest);
	cc1101_spi_write_reg(CC1101_REG_PTEST, config->ptest);
	cc1101_spi_write_reg(CC1101_REG_AGCTEST, config->agctest);
	cc1101_spi_write_reg(CC1101_REG_TEST2, config->test2);
	cc1101_spi_write_reg(CC1101_REG_TEST1, config->test1);
	cc1101_spi_write_reg(CC1101_REG_TEST0, config->test0);
}
