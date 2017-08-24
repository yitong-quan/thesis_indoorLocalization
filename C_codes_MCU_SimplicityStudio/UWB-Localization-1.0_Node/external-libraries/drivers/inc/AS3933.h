#ifndef DRIVERS_AS3933_H_
#define DRIVERS_AS3933_H_

#include "config.h"
#include "revisionDef.h"

#include <stdbool.h>
#include <stdint.h>

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "TIMER.h"
#include "CRC.h"

volatile bool WAKEUP_received;
volatile bool WAKEUP_active;
volatile bool WAKEUP_24BIT;

#if		(REVISION == IDENTIFIER_REV_1) || (REVISION == IDENTIFIER_REV_2)

	#define AS3933_USART				USART0
	#define AS3933_USART_CLK			cmuClock_USART0
	#define AS3933_USART_LOC			USART_ROUTE_LOCATION_LOC0

	#define AS3933_MOSI_PORT			gpioPortE
	#define AS3933_MOSI_PIN 			10

	#define AS3933_MISO_PORT 			gpioPortE
	#define AS3933_MISO_PIN 			11

	#define AS3933_CLK_PORT 			gpioPortE
	#define AS3933_CLK_PIN				12

	#define AS3933_CS_PORT 				gpioPortE
	#define AS3933_CS_PIN 				13

	#define AS3933_WAKE_PORT			gpioPortA
	#define AS3933_WAKE_PIN				0

	#define AS3933_DCLK_PORT			gpioPortC
	#define AS3933_DCLK_PIN				1

	#define AS3933_DATA_PORT			gpioPortC
	#define AS3933_DATA_PIN				0

#elif	(REVISION == IDENTIFIER_REV_3) || (REVISION == TABLE_REV_1)

	#define AS3933_USART				USART0
	#define AS3933_USART_CLK			cmuClock_USART0
	#define AS3933_USART_LOC			USART_ROUTE_LOCATION_LOC0

	#define AS3933_MOSI_PORT			gpioPortE
	#define AS3933_MOSI_PIN 			10

	#define AS3933_MISO_PORT 			gpioPortE
	#define AS3933_MISO_PIN 			11

	#define AS3933_CLK_PORT 			gpioPortE
	#define AS3933_CLK_PIN				12

	#define AS3933_CS_PORT 				gpioPortC
	#define AS3933_CS_PIN 				1

	#define AS3933_WAKE_PORT			gpioPortE
	#define AS3933_WAKE_PIN				13

	#define AS3933_DCLK_PORT			gpioPortC
	#define AS3933_DCLK_PIN				0

	#define AS3933_DATA_PORT			gpioPortA
	#define AS3933_DATA_PIN				0

#endif

// SPI command
#define AS3933_WRITE_REG  				0x00
#define AS3933_READ_REG   				0x40
#define AS3933_DIRECT_CMD 				0xc0

/* Direct commands (see AS3933 datasheet, p. 17) */
#define AS3933_CMD_CLEAR_WAKE			0x00
#define AS3933_CMD_RESET_RSSI			0x01
#define AS3933_CMD_TRIM_OSC				0x02
#define AS3933_CMD_CLEAR_FALSE			0x03
#define AS3933_CMD_PRESET_DEFAULT		0x04

// Register addresses
#define AS3933_REG0      				0x00
#define AS3933_REG1        				0x01
#define AS3933_REG2        				0x02
#define AS3933_REG3        				0x03
#define AS3933_REG4        				0x04
#define AS3933_REG5        				0x05
#define AS3933_REG6       				0x06
#define AS3933_REG7        				0x07
#define AS3933_REG8        				0x08
#define AS3933_REG9        				0x09
#define AS3933_REG10       				0x0a
#define AS3933_REG11       				0x0b
#define AS3933_REG12       				0x0c
#define AS3933_REG13       				0x0d
#define AS3933_REG14       				0x0e
#define AS3933_REG15       				0x0f
#define AS3933_REG16       				0x10
#define AS3933_REG17       				0x11
#define AS3933_REG18       				0x12
#define AS3933_REG19       				0x13

// R0
#define AS3933_PATT32      				0x80
#define AS3933_DAT_MASK    				0x40
#define AS3933_ON_OFF      				0x20
#define AS3933_MUX_123     				0x10
#define AS3933_EN_A2       				0x08
#define AS3933_EN_A3       				0x04
#define AS3933_EN_A1       				0x02

// R1
#define AS3933_ABS_HY      				0x80
#define AS3933_AGC_TLIM    				0x40
#define AS3933_AGC_UD      				0x20
#define AS3933_ATT_ON      				0x10
#define AS3933_EN_MANCH    				0x08
#define AS3933_EN_PAT2     				0x04
#define AS3933_EN_WPAT     				0x02
#define AS3933_EN_XTAL     				0x01

// R2
#define AS3933_S_ABS       				0x80
#define AS3933_AGC_TLIM    				0x40
#define AS3933_G_BOOST     				0x20
#define AS3933_DISPLAY_CLK 				0x0c
#define AS3933_S_WU1_REL   				0x00
#define AS3933_S_WU1_TIG   				0x01
#define AS3933_S_WU1_STR   				0x02

// R3
#define AS3933_HY_20m      				0x80
#define AS3933_HY_POS      				0x40
#define AS3933_FS_SCL_080  				0x00
#define AS3933_FS_SCL_115  				0x08
#define AS3933_FS_SCL_155  				0x10
#define AS3933_FS_SCL_190  				0x18
#define AS3933_FS_SCL_230  				0x20
#define AS3933_FS_SCL_265  				0x28
#define AS3933_FS_SCL_300  				0x30
#define AS3933_FS_SCL_350  				0x38
#define AS3933_FS_ENV_4096 				0x00
#define AS3933_FS_ENV_2184 				0x01
#define AS3933_FS_ENV_1490 				0x02
#define AS3933_FS_ENV_1130 				0x03
#define AS3933_FS_ENV_910  				0x04
#define AS3933_FS_ENV_762  				0x05
#define AS3933_FS_ENV_655  				0x06
#define AS3933_FS_ENV_512  				0x07

// R4
#define AS3933_T_OFF_1MS   				0x00
#define AS3933_T_OFF_2MS   				0x40
#define AS3933_T_OFF_4MS   				0x80
#define AS3933_T_OFF_8MS   				0xc0
#define AS3933_D_RES_1k    				0x00
#define AS3933_D_RES_3k    				0x10
#define AS3933_D_RES_9k    				0x20
#define AS3933_D_RES_27k   				0x30
#define AS3933_GR_0        				0x00
#define AS3933_GR_4        				0x04
#define AS3933_GR_8        				0x06
#define AS3933_GR_12       				0x08
#define AS3933_GR_16       				0x0A
#define AS3933_GR_20       				0x0C
#define AS3933_GR_24       				0x0E

// R5 2nd Byte of WakeUp Pattern

// R6 1st Byte of WakeUp Pattern

// R7
#define AS3933_T_OUT_0     				0x00
#define AS3933_T_OUT_50    				0x20
#define AS3933_T_OUT_100   				0x40
#define AS3933_T_OUT_150   				0x60
#define AS3933_T_OUT_200  				0x80
#define AS3933_T_OUT_250   				0xa0
#define AS3933_T_OUT_300   				0xc0
#define AS3933_T_OUT_350   				0xe0
#define AS3933_T_HBIT_8192 				0x03
#define AS3933_T_HBIT_6552 				0x04
#define AS3933_T_HBIT_5460 				0x05
#define AS3933_T_HBIT_4680 				0x06
#define AS3933_T_HBIT_4096 				0x07
#define AS3933_T_HBIT_3640 				0x08
#define AS3933_T_HBIT_3276 				0x09
#define AS3933_T_HBIT_2978 				0x0a
#define AS3933_T_HBIT_2730 				0x0b
#define AS3933_T_HBIT_2520 				0x0c
#define AS3933_T_HBIT_2340 				0x0d
#define AS3933_T_HBIT_2184 				0x0e
#define AS3933_T_HBIT_2048 				0x0f
#define AS3933_T_HBIT_1926 				0x10
#define AS3933_T_HBIT_1820 				0x11
#define AS3933_T_HBIT_1724 				0x12
#define AS3933_T_HBIT_1638 				0x13
#define AS3933_T_HBIT_1560 				0x14
#define AS3933_T_HBIT_1488 				0x15
#define AS3933_T_HBIT_1424 				0x16
#define AS3933_T_HBIT_1364 				0x17
#define AS3933_T_HBIT_1310 				0x18
#define AS3933_T_HBIT_1260 				0x19
#define AS3933_T_HBIT_1212 				0x1a
#define AS3933_T_HBIT_1170 				0x1b
#define AS3933_T_HBIT_1128 				0x1c
#define AS3933_T_HBIT_1092 				0x1d
#define AS3933_T_HBIT_1056 				0x1e
#define AS3933_T_HBIT_1024 				0x1f

// R8
#define AS3933_BAND_95_150_kHz			0x00
#define AS3933_BAND_65_95_kHz			0x20
#define AS3933_BAND_40_65_kHz			0x40
#define AS3933_BAND_23_40_kHz			0x60
#define AS3933_BAND_15_23_kHz			0xe0
#define AS3933_T_AUTO_0    				0x00
#define AS3933_T_AUTO_1s   				0x01
#define AS3933_T_AUTO_5s   				0x02
#define AS3933_T_AUTO_20s  				0x03
#define AS3933_T_AUTO_2m   				0x04
#define AS3933_T_AUTO_15m  				0x05
#define AS3933_T_AUTO_1h   				0x06
#define AS3933_T_AUTO_2h   				0x07

// R9
#define AS3933_BLOCK_AGC   				0x80

// R10 RSSI1 Ch1
// R11 RSSI2 Ch2
// R12 RSSI3 Ch3
#define AS3933_RSSI        				0x1f

// R13 FWAKE

//R14
#define AS3933_RC_CAL_OK   				0x80
#define AS3933_RC_CAL_KO   				0x40
// TODO: RC_OSC_TAPS

// R15
#define AS3933_LC_OSC_OK   				0x10
#define AS3933_LC_OSC_KO   				0x08

// R16
#define AS3933_CLOCK_GEN_DIS 			0x80
#define AS3933_RC_OSC_MIN	 			0x20
#define AS3933_RC_OSC_MAX	 			0x10
// TODO: LC_OSC_MUX

//R17 CAP_CH1
//R18 CAP_CH2
//R19 CAP_CH3

/*
 *
 */
void AS3933_spi_init(uint32_t SPI_BAUDRATE);

/**
 * Writes the given data at a single configuration register of the AS3933.
 */
void AS3933_spi_write_reg(uint8_t address, uint8_t data);

/**
 * Writes data to multiple configuration registers. The first register being at
 * the given address. The index of the buffer and the address are incremented
 * sequentially until the given number of writes have been performed.
 */
void AS3933_spi_write_burst_reg(uint8_t address, const uint8_t *buffer,
		uint8_t count);

/**
 * Reads a single configuration register at the given address and returns the
 * value.
 */
uint8_t AS3933_spi_read_reg(uint8_t address);

/**
 *
 */
void AS3933_spi_direct_cmd(uint8_t cmd);

/**
 *
 */bool AS3933_calibrate(uint32_t SPI_BAUDRATE);

/**
 *
 */
void AS3933_manchester_wakeup_pattern(uint8_t wakeup_node_id);

/**
 *
 */
void AS3933_setup(bool manchester, bool ADDRESS_16BIT);

/**
 *
 */
void AS3933_wakeup_init(bool config_interrupt);

/**
 *
 */
void AS3933_wakeup_int_enable(void);

/**
 *
 */
void AS3933_wakeup_int_disable(void);

/*
 *
 */
void AS3933_prepare_wakeup(uint32_t SPI_BAUDRATE, bool USE_16bit);

/*
 *
 */
void AS3933_start_wakeup(void);

/*
 *
 */
void AS3933_enable_wakeup_by_8bit_address(uint8_t address,
		uint32_t SPI_BAUDRATE);

/*
 *
 */
void AS3933_enable_wakeup_by_16bit_address(uint16_t address,
		uint32_t SPI_BAUDRATE);

/*
 *
 */
void AS3933_enable_wakeup_by_frequency_detection(uint32_t SPI_BAUDRATE);

/*
 *
 */
void AS3933_enable_wakeup_by_24bit_address(uint32_t address,
		uint32_t SPI_BAUDRATE);


/*
 *
 */
bool AS3933_receive_data(uint32_t *woke_up_by_ID, uint8_t * options, uint16_t *n_measurements, uint32_t SPI_BAUDRATE);

/*
 *
 */
void AS3933_EFM_sleep_enable_wake_up (uint32_t WAKEUP_ID);

#endif /* DRIVERS_AS3933_H_ */
