#ifndef DRIVERS_CC1101_H_
#define DRIVERS_CC1101_H_

#include "config.h"
#include "revisionDef.h"
#include <stdint.h>

#include "CC1101_def.h"

#if		REVISION == IDENTIFIER_REV_1

	#define CC1101_GDO0_PORT 	gpioPortC
	#define CC1101_GDO0_PIN 	14

	#define CC1101_GDO2_PORT 	gpioPortC
	#define CC1101_GDO2_PIN 	15

	#define CC1101_CS_PORT 		gpioPortD
	#define CC1101_CS_PIN 		5

	#define CC1101_MOSI_PORT	gpioPortE
	#define CC1101_MOSI_PIN 	10

	#define CC1101_MISO_PORT 	gpioPortE
	#define CC1101_MISO_PIN 	11

	#define CC1101_CLK_PORT 	gpioPortE
	#define CC1101_CLK_PIN		12

#elif	REVISION == IDENTIFIER_REV_2

	#define CC1101_GDO0_PORT 	gpioPortC
	#define CC1101_GDO0_PIN 	14

	#define CC1101_GDO2_PORT 	gpioPortC
	#define CC1101_GDO2_PIN 	15

	#define CC1101_CS_PORT 		gpioPortB
	#define CC1101_CS_PIN 		14

	#define CC1101_MOSI_PORT	gpioPortE
	#define CC1101_MOSI_PIN 	10

	#define CC1101_MISO_PORT 	gpioPortE
	#define CC1101_MISO_PIN 	11

	#define CC1101_CLK_PORT 	gpioPortE
	#define CC1101_CLK_PIN		12

#elif	REVISION == IDENTIFIER_REV_3

	#define CC1101_GDO0_PORT 	gpioPortC
	#define CC1101_GDO0_PIN 	14

	#define CC1101_GDO2_PORT 	gpioPortC
	#define CC1101_GDO2_PIN 	15

	#define CC1101_CS_PORT 		gpioPortB
	#define CC1101_CS_PIN 		14

	#define CC1101_MOSI_PORT	gpioPortE
	#define CC1101_MOSI_PIN 	10

	#define CC1101_MISO_PORT 	gpioPortE
	#define CC1101_MISO_PIN 	11

	#define CC1101_CLK_PORT 	gpioPortE
	#define CC1101_CLK_PIN		12

#elif	REVISION == TABLE_REV_1

	#define CC1101_GDO0_PORT 	gpioPortC
	#define CC1101_GDO0_PIN 	14

	#define CC1101_GDO2_PORT 	gpioPortC
	#define CC1101_GDO2_PIN 	15

	#define CC1101_CS_PORT 		gpioPortB
	#define CC1101_CS_PIN 		13

	#define CC1101_MOSI_PORT	gpioPortE
	#define CC1101_MOSI_PIN 	10

	#define CC1101_MISO_PORT 	gpioPortE
	#define CC1101_MISO_PIN 	11

	#define CC1101_CLK_PORT 	gpioPortE
	#define CC1101_CLK_PIN		12

#elif	(REVISION == BASE_REV_1) || (REVISION == BASE_REV_2)

	#define CC1101_GDO0_PORT 	gpioPortC
	#define CC1101_GDO0_PIN 	14

	#define CC1101_GDO2_PORT 	gpioPortC
	#define CC1101_GDO2_PIN 	15

	#define CC1101_CS_PORT 		gpioPortD
	#define CC1101_CS_PIN 		5

	#define CC1101_MOSI_PORT	gpioPortE
	#define CC1101_MOSI_PIN 	10

	#define CC1101_MISO_PORT 	gpioPortE
	#define CC1101_MISO_PIN 	11

	#define CC1101_CLK_PORT 	gpioPortE
	#define CC1101_CLK_PIN		12

#endif

struct cc1101_config {
	/* GDO2 output pin configuration. */
	uint8_t iocfg2;

	/* GDO1 output pin configuration. */
	uint8_t iocfg1;

	/* GDO0 output pin configuration. */
	uint8_t iocfg0;

	/* RX FIFO and TX FIFO thresholds. */
	uint8_t fifothr;

	/* Sync word (high and low byte). */
	uint8_t sync1;
	uint8_t sync0;

	/* Packet length. */
	uint8_t pktlen;

	/* Packet automation control. */
	uint8_t pktctrl1;
	uint8_t pktctrl0;

	/* Device address. */
	uint8_t addr;

	/* Channel number. */
	uint8_t channr;

	/* Frequency synthesizer control. */
	uint8_t fsctrl1;
	uint8_t fsctrl0;

	/* Frequency control word (high, middle and low byte). */
	uint8_t freq2;
	uint8_t freq1;
	uint8_t freq0;

	/* Modem configuration. */
	uint8_t mdmcfg4;
	uint8_t mdmcfg3;
	uint8_t mdmcfg2;
	uint8_t mdmcfg1;
	uint8_t mdmcfg0;

	/* Modem deviation setting. */
	uint8_t deviatn;

	/* Main Radio Control State Machine configuration. */
	uint8_t mcsm2;
	uint8_t mcsm1;
	uint8_t mcsm0;

	/* Frequency Offset Compensation configuration. */
	uint8_t foccfg;

	/* Bit Synchronization configuration. */
	uint8_t bscfg;

	/* AGC control. */
	uint8_t agcctrl2;
	uint8_t agcctrl1;
	uint8_t agcctrl0;

	/* Event 0 timeout (high and low byte). */
	uint8_t worevt1;
	uint8_t worevt0;

	/* Wake On Radio control. */
	uint8_t worctrl;

	/* Front end RX configuration. */
	uint8_t frend1;

	/* Front end TX configuration. */
	uint8_t frend0;

	/* Frequency synthesizer calibration. */
	uint8_t fscal3;
	uint8_t fscal2;
	uint8_t fscal1;
	uint8_t fscal0;

	/* RC oscillator configuration. */
	uint8_t rcctrl1;
	uint8_t rcctrl0;

	/* Frequency synthesizer calibration control. */
	uint8_t fstest;

	/* Production test. */
	uint8_t ptest;

	/* AGC test. */
	uint8_t agctest;

	/* Various test settings. */
	uint8_t test2;
	uint8_t test1;
	uint8_t test0;
};


void cc1101_spi_init(uint32_t SPI_BAUDRATE);

/**
 * Power-Up Reset
 */
void cc1101_spi_powerup_reset(void);


/**
 * Writes the given data at a single configuration register of the CC1101.
 */
void cc1101_spi_write_reg(uint8_t address, uint8_t data);


/**
 * Writes data to multiple configuration registers. The first register being at
 * the given address. The index of the buffer and the address are incremented
 * sequentially until the given number of writes have been performed.
 */
void cc1101_spi_write_burst_reg(uint8_t address, const uint8_t * const buffer,
				uint8_t count);


/**
 * Reads a single configuration register at the given address and returns the
 * value.
 */
uint8_t cc1101_spi_read_reg(uint8_t address);


/**
 * Reads from multiple configuration registers. The first register being at
 * the given address. The index of the buffer and the address are incremented
 * sequentially until the given number of registers have been read.
 */
void cc1101_spi_read_burst_reg(uint8_t address, uint8_t * const buffer,
		uint8_t count);


/**
 * Writes to the special strobe register at the given address.
 */
uint8_t cc1101_spi_strobe(uint8_t cmd);


/**
 *
 */
void cc1101_spi_write_config(const struct cc1101_config *config);


#endif /* DRIVERS_CC1101_H_ */
