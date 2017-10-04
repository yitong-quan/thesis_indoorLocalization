/*
 * DWM1000.c
 *
 *  Created on: 03.05.2016
 *      Author: Patrick Fehrenbach
 */
#include "DWM1000.h"
#include "config.h"
#include "DWM1000_config_param.h"
#include "enum.h"

extern uint8_t POLL_message[] ;								// Poll message
extern uint8_t RESP_message[] ;								// Response message
extern uint8_t FINAL_message[]  ;   						// Final message
uint8_t test =0;

enum states_DWM1000 DWM1000_deviceMode = STATE_IDLE;
extern enum states_tag state;
extern enum states_node state1;
extern enum states_localization localization;

volatile dwt_local_data_t DWM1000local ; 					// Static local device data

/* Set the GPIO mode: Every GPIO pin operates in default mode as GPIO (value 0x00)
 * 					  0x01 is the second mode of the GPIO pin, this mode depends on the used pin
 */

void DWM1000_GPIO_set_mode(uint8_t gpio_pin, uint32_t mode)
{
	uint32_t mode_reg = 0;
	uint32_t gpio_reg = 0;

	switch (gpio_pin)
	{
		case GPIO_PIN0:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN1:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN2:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN3:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN4:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN5:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN6:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN7:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		case GPIO_PIN8:
			mode_reg = mode & GPIO_MODE_MASK;
			break;
		default:
			break;
	}

	gpio_reg= DWM1000_read32bitoffsetregister(GPIO_CTRL_ID, GPIO_MODE_OFFSET);

	DWM1000_write32bitoffsetregister(GPIO_CTRL_ID, GPIO_MODE_OFFSET, mode_reg | gpio_reg);

}

void DWM1000_set_GPIOs_sec_mode()
{
	DWM1000_GPIO_set_mode(GPIO_PIN0, PIN0_RXOKLED_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN1, PIN1_SFDLED_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN2, PIN2_RXLED_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN3, PIN3_TXLED_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN5, PIN5_EXTTXE_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN6, PIN6_EXTRXE_MODE);

	DWM1000_write32bitoffsetregister(PMSC_ID, PMSC_LEDC_OFFSET, PMSC_LEDC_BLNKEN | 0x0020);
}

void DWM1000_set_GPIO_mode()
{
	DWM1000_GPIO_set_mode(GPIO_PIN0, GPIO_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN1, GPIO_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN2, GPIO_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN3, GPIO_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN5, GPIO_MODE);
	DWM1000_GPIO_set_mode(GPIO_PIN6, GPIO_MODE);
}

void DWM1000_spi_default()
{
	GPIO_PinModeSet(DWM1000_CS_PORT, DWM1000_CS_PIN, gpioModePushPull, true);
}


void DWM1000_spi_init(uint32_t SPI_BAUDRATE)
{

	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(DWM1000_MOSI_PORT, DWM1000_MOSI_PIN, gpioModePushPull, false);
	GPIO_PinModeSet(DWM1000_MISO_PORT, DWM1000_MISO_PIN, gpioModeInput, false);
	GPIO_PinModeSet(DWM1000_CLK_PORT, DWM1000_CLK_PIN, gpioModePushPull, false);

	/*
	 * Configure the chip select pin as output and set it to ensure that the
	 * chip is not active.
	 */
	GPIO_PinModeSet(DWM1000_CS_PORT, DWM1000_CS_PIN, gpioModePushPull, true);
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

uint8_t DWM1000_init()
{
	//DWM1000local.dblbuffon = 0; 														// Double buffer mode off by default
	//DWM1000local.prfIndex = 0; 														// 16MHz
	//DWM1000local.cdata.aatset = 0; 													// Auto ACK bit not set
	//DWM1000local.wait4resp = 0;
	//DWM1000local.sleep_mode = 0;
	//
	//DWM1000local.dwt_txcallback = NULL;
	//DWM1000local.dwt_rxcallback = NULL;

	//uint16_t OTP_addr = 0;
	uint32_t ldoTune = 0;

	//begin
	RTC_delay_ms(5);

	//select
	DWM1000_enable_clocks(ENABLE_ALL_SEQ);												// clock for sequencing, try locking PLL speed
	RTC_delay_ms(5);

	DWM1000_short_reset();																// hard reset
	while (!(GPIO_PinInGet(DWM1000_RSTn_PORT, DWM1000_RSTn_PIN)));

	DWM1000_write32bitoffsetregister(PANADR_ID, 0, 0x0000FF);							// set PAN ID

	DWM1000local.sysCFGreg = SYS_CFG_HIRQ_POL | SYS_CFG_DIS_DRXB;						// set SYS_CFG reg
	DWM1000_write32bitoffsetregister(SYS_CFG_ID, 0, DWM1000local.sysCFGreg);

	DWM1000_clear_irq_mask_register();													// clear event mask register

	//Load LDE micro code
	DWM1000_enable_clocks(FORCE_SYS_XTI);												// set clock to XTI clock
	RTC_delay_ms(5);

	DWM1000local.partID = DWM1000_read16bitoffsetregister(OTP_IF_ID, OTP_CTRL);			// read the Part ID

	ldoTune = DWM1000_read_OTP(OTP_ADDR);

	if((ldoTune & 0xFF) != 0)															// Load LDO tune from OTP and kick it if there is a value actually programmed
	{
		//uint8_t ldok = OTP_SF_LDO_KICK;												// Kick LDO tune
		//DWM1000_spi_write_burst_reg(OTP_IF_ID, OTP_SF, &ldok, 1); 					// Set load LDE kick bit
		//DWM1000local.sleep_mode |= AON_WCFG_ONW_LLDO; 								// LDO tune must be kicked at wake-up
	}

	DWM1000local.pmscreg = DWM1000_read32bitoffsetregister(PMSC_ID, PMSC_CTRL0_OFFSET);
	DWM1000local.partID = DWM1000_read16bitoffsetregister(OTP_IF_ID, OTP_CTRL);			// read the Part ID

	DWM1000_loaducodefromrom();
	//DWM1000local.sleep_mode |= AON_WCFG_ONW_LLDE; 									// microcode must be loaded at wake-up


	//OTP_addr = DWM1000_read_OTP(XTRIM_ADDRESS) & 0xFFFF;								// Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
	//DWM1000local.otprev = (OTP_addr >> 8) & 0xFF;										// OTP revision is next byte

	//DWM1000local.xtrim = OTP_addr & 0x1F;
	//if (!DWM1000local.xtrim) 															// A value of 0 means that the crystal has not been trimmed
	//{
		//DWM1000local.xtrim = FS_XTALT_MIDRANGE ; 										// Set to mid-range if no calibration value inside
	//}
	// Configure XTAL trim
	//DWM1000_XTALtrim(DWM1000local.xtrim);

	DWM1000_idle_mode();																// set to idle mode, tranceiver off
	DWM1000local.panID = DWM1000_read32bitoffsetregister(PANADR_ID, 0); 				// read PAN ID

	DWM1000local.sysCFGreg = DWM1000_read32bitoffsetregister(SYS_CFG_ID, 0);			// read SYSCFG
	DWM1000local.chancontrreg = DWM1000_read32bitoffsetregister(CHAN_CTRL_ID, 0);		// read CHAN CTRL reg
	DWM1000local.txFCTRL = DWM1000_read32bitoffsetregister(TX_FCTRL_ID, 0);				// read TX FCTRL reg

	DWM1000local.sysmaskctrl = DWM1000_read32bitoffsetregister(SYS_MASK_ID, 0);			// read System event mask reg

	DWM1000local.deviceID = DWM1000_get_device_ID();									// Read the device ID

	if (DWT_DEVICE_ID != DWM1000local.deviceID) 										// if the device ID read is not the right ID => error
	{
		return DWT_ERROR;
	}

	return DWT_SUCCESS;
}

void DWM1000_XTALtrim(uint8_t trimvalue)
{
	uint8_t write_buffer;

	DWM1000_spi_read_burst_reg(FS_CTRL_ID,FS_XTALT_OFFSET, &write_buffer, 1);

	write_buffer &= ~FS_XTALT_MASK ;

	write_buffer |= (FS_XTALT_MASK & trimvalue) ; 										// We should not change high bits, cause it will cause malfunction

	DWM1000_spi_write_burst_reg(FS_CTRL_ID, FS_XTALT_OFFSET, &write_buffer, 1);
}


void DWM1000_configuration(volatile dwt_config_t *config)
{
	uint8_t nsSfd_result  = 0;
	uint8_t useDWnsSFD = 0;
	uint32_t regval;
	uint16_t lde_coeff = get_lde_replicaCoeff(config->rxCode);
	uint8_t prfIndex = DWM1000local.prfIndex = config->prf - DWT_PRF_16M;
	uint8_t chan = config->chan;
	uint16_t sfdTO = config->sfdTO;

/* Set the data rate (Mbps) (110 kbps, 850 kbps, 6.8 Mbps):
 * - set it to 110 kbps we have to set RXM110K bit in 0x04 register (System Configuration) to 1 otherwise clear this bit to 0
 * - set also the LDE Replica Coeffiecient depending on RX_PCODE (here config->rxcode) (depends on channel number and PRF settings see s. 203 table 58)
 * 		- for 110 kbps divide the replica coeffiecent by 8
 * - set the PHR mode: Set PHR_MODE bits (17,16) in 0x04 register (System Configuration)
 * 		- PHR_Mode: 00: Standard Frame mode. Use this setting is for IEEE 802.15.4 compliance
 * 					11: Long Frames mode. Proprietary PHR encoding. Frame Length 0-1023. not IEEE standard (not allowed)
 */
	if(DWT_BR_110K == config->dataRate)
	{
		DWM1000local.sysCFGreg |= SYS_CFG_RXM110K;										// set RXM110K to 1 and safe it in DWM1000local.sysCFGreg
	    lde_coeff >>= 3; 																// lde_replicaCoeff must be divided by 8
	}
	else
	{
	    DWM1000local.sysCFGreg &= (~SYS_CFG_RXM110K);									// keep RXM110K bit low to get 850 kbps or 6.8 Mbps
	}

	if(config->dataRate == DWT_BR_6M8)
	{
		DWM1000local.sfdlength = 0x08;
	}
	else if(config->dataRate == DWT_BR_850K)
	{
		DWM1000local.sfdlength = 0x10;
	}
	else
	{
		DWM1000local.sfdlength = 0x40;
	}

	DWM1000_spi_write_burst_reg(USR_SFD_ID, 0x00, &DWM1000local.sfdlength, 1);

	DWM1000local.longFrames = config->phrMode ;
	DWM1000local.sysCFGreg |= (SYS_CFG_PHR_MODE_11 & (config->phrMode << 16));			// depending on config->phrMode settings bit masking of PHR_MODE bits

	DWM1000_write32bitoffsetregister(SYS_CFG_ID, 0x00, DWM1000local.sysCFGreg);			// set PHR-Mode in 0x04 register, set the hole 0x04 register

	DWM1000_write16bitoffsetregister(LDE_IF_ID, LDE_REPC_OFFSET, lde_coeff);			// Set the LDE Replica Coeffiecient in 0x2E register

	configure_lde(prfIndex);															// 16-bit LDE configuration tuning register

	set_FS_PLL_config_tune(chan);														// Configure frequency synthesiser config and tune

	set_rx_ctrl(chan);																	// Configure RF RX blocks (for specified channel/bandwidth)

	set_tx_ctrl(chan);																	// Configure RF TX blocks (for specified channel and PRF)
    																					// Configure RF TX control
// todo configure tx rf (dwt_txconfig_t), power (0x1E), Pulse Generator delay (0x2A)
	set_TC_PGDELAY(chan);																// Configure Pulse Generator Delay value
	set_TX_Power_config(prfIndex, chan,config-> smarttxpower);


	// Configure the baseband parameters
	set_drx_tune0b(config->dataRate, config->nsSFD);									// Configure the baseband parameter DTUNE0 (for specified data rate and SFD settings)
	set_drx_tune1a(prfIndex);															// Configure the baseband parameter DTUNE1a (register 0x27:04) (for specified PRF)
	set_drx_tune1b_4H(config->dataRate, config->txPreambLength);
	set_drx_tune2(prfIndex, config->rxPAC);
	//set_drx_tune3(sfdTO);																// set SFD timeout

// Configure AGC parameters
	set_agc_tune1 (prfIndex);															// AGC Tuning register 1
	DWM1000_write32bitoffsetregister(AGC_CFG_STS_ID, AGC_TUNE2_OFFSET, AGC_TUNE2_VAL);	// AGC Tuning register 2 0x23:0C. write value 0x2502A907 (s.115 User Manual)
	DWM1000_write16bitoffsetregister(AGC_CFG_STS_ID, AGC_TUNE3_OFFSET, AGC_TUNE3_VAL);  // AGC Tuning register 3 0x23:12. write value 0x0035 (s.115 User Manual)

// Set (non-standard) user SFD for improved performance
	if(config->nsSFD)
	{
		set_sfd_improved(config->dataRate);												// Write non standard (DW) SFD length
	    nsSfd_result = 3 ;
	    useDWnsSFD = 1 ;
	}

/* Channel Control:
 * Channel Control Register 0x1F. This register is 4 bytes long. Its use to select transmit and receive channels,
 * and configure preamble codes and some related parameters.
 * Bits 0-3: 	TX Channel; Bits 4-7: RX Channel; Bit 17: DWSFD (enables non- standard SFD sequence)
 * Bits 18-19: 	RXPRF (01: 16 MHz PRF, 10: 64 MHz PRF, PRF used in receiver)
 * Bit 20: 		TNSSFD (enables non- standard SFD sequence in transmitter)
 * Bit 21: 		RNSSFD (enables non- standard SFD sequence in receiver)
 * Bit 22-26: 	TX PCODE (selects Preamble code used in transmitter)
 * Bit 27-31: 	RX PCODE (selects Preamble code used in receiver)
 *
 */
	regval = (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | 						// Transmit Channel
	         (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | 						// Receive Channel
	         (CHAN_CTRL_RXFPRF_MASK & (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | 					// RX PRF
	         ((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | 	// nsSFD enable RX&TX
	         (CHAN_CTRL_DWSFD & (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | 							// Use DW nsSFD
	         (CHAN_CTRL_TX_PCOD_MASK & (config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | 				// TX Preamble Code
	         (CHAN_CTRL_RX_PCOD_MASK & (config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)) ; 				// RX Preamble Code

	DWM1000_write32bitoffsetregister(CHAN_CTRL_ID,0,regval);

/* Transmit Frame Control (0x08):
 * This register contains a number of TX control fields.
 * Bits 0-6: 	TFLEN (Transmit frame length, can be up to 127 bytes long)
 * Bits 7-9: 	TFLE (non-standard mode up to 1023 bytes)
 * Bits 13-14: 	TXBR (Transmit Bit Rate)
 * Bit 15: 		TR (Transmit Ranging enable: Ranging bit)
 * Bits 16-17: 	TXPRF (Transmit Pulse Repetition Frequency, PRF used in transmitter)
 * Bits 18-19: 	TXPSR (Transmit Preamble Symbol Repetitions (PSR))
 *
 */
	// Set up TX Preamble Size and TX PRF
	// Set up TX Ranging Bit and Data Rate
	DWM1000local.txFCTRL = (config->txPreambLength | config->prf) << 16;
	DWM1000local.txFCTRL |= (config->dataRate << TX_FCTRL_TXBR_SHFT) | TX_FCTRL_TR; 				// Always set ranging bit !!!
	DWM1000_write32bitoffsetregister(TX_FCTRL_ID,0,DWM1000local.txFCTRL);
}


void set_TX_Power_config(uint8_t prfindex, uint8_t chan, uint8_t smarttxpower)
{
	uint32_t tx_power;
	if(smarttxpower)
	{
		if(prfindex)																					// 64 MHZ PRF
		{
			switch(chan)
			{
				case 1:
					tx_power = 0x07274767UL;
					break;
				case 2:
					tx_power = 0x07274767UL;
					break;
				case 3:
					tx_power = 0x2B4B6B8BUL;
					break;
				case 4:
					tx_power = 0x3A5A7A9AUL;
					break;
				case 5:
					tx_power = 0x25456585UL;
					break;
				case 7:
					tx_power = 0x5171B1D1UL;
					break;
				default:
					break;
			}
		}
		else																							// 16 MHZ PRF
		{
			switch(chan)
			{
				case 1:
					tx_power = 0x15355575UL;
					break;
				case 2:
					tx_power = 0x15355575UL;
					break;
				case 3:
					tx_power = 0x0F2F4F6FUL;
					break;
				case 4:
					tx_power = 0x1F1F3F5FUL;
					break;
				case 5:
					tx_power = 0x0E082848UL;
					break;
				case 7:
					tx_power = 0x32527292UL;
					break;
				default:
					break;
			}
		}
	}
	else
	{
		if(prfindex)																					// 64 MHZ PRF
		{
			switch(chan)
			{
 	 			case 1:
 	 				tx_power = 0x67676767UL;
 	 				break;
 	 			case 2:
 	 				tx_power = 0x67676767UL;
 	 				break;
 	 			case 3:
 	 				tx_power = 0x8B8B8B8BUL;
 	 				break;
 	 			case 4:
 	 				tx_power = 0x9A9A9A9AUL;
 	 				break;
 	 			case 5:
 	 				tx_power = 0x85858585UL;
 	 				break;
 	 			case 7:
 	 				tx_power = 0xD1D1D1D1UL;
 	 				break;
 	 			default:
 	 				break;
			}
		}
		else																							// 16 MHZ PRF
		{
			switch(chan)
			{
 	 			case 1:
 	 				tx_power = 0x75757575UL;
 	 				break;
 	 			case 2:
 	 				tx_power = 0x75757575UL;
 	 				break;
 	 			case 3:
 	 				tx_power = 0x6F6F6F6FUL;
 	 				break;
 	 			case 4:
 	 				tx_power = 0x5F5F5F5FUL;
 	 				break;
 	 			case 5:
 	 				tx_power = 0x48484848UL;
 	 				break;
 	 			case 7:
 	 				tx_power = 0x92929292UL;
 	 				break;
 	 			default:
 	 				break;
			}
		}
	}
	DWM1000_write32bitoffsetregister(TX_POWER_ID, 0, tx_power);
}


/* Transmitter Calibration block
 * Transmitter Calibration –Pulse Generator Delay register 0x2A:0B. This is a 8-bit sub-register. Its a configuration register for
 * setting the Pulse Generator Delay value.
 * TC_PGDELAY values: TX Channel: 	1: 0xC9          (user manual p. 146 table 37)
 * 									2: 0xC2
 * 									3: 0xC5
 * 									4: 0x95
 * 									5: 0xC0
 * 									7: 0x93
 *
 */


void set_TC_PGDELAY (uint8_t chan)
{
	uint8_t tx_pgdelay;

	switch (chan)
	{
		case 1:
			tx_pgdelay = TC_PGDELAY_CH1;
			break;
		case 2:
			tx_pgdelay = TC_PGDELAY_CH2;
			break;
		case 3:
			tx_pgdelay = TC_PGDELAY_CH3;
			break;
		case 4:
			tx_pgdelay = TC_PGDELAY_CH4;
			break;
		case 5:
			tx_pgdelay = TC_PGDELAY_CH5;
			break;
		case 7:
			tx_pgdelay = TC_PGDELAY_CH7;
			break;
		default:
			break;
	}

	DWM1000_spi_write_burst_reg(TX_CAL_ID, TC_PGDELAY_OFFSET, &tx_pgdelay, 1);
}


/* User defined SFD sequence
 * User-specified short/long TX/RX SFD sequences 0x21 register
 * The length of an SFD sequence for 110 kbps is always 64 symbols, but the length of the SFD sequence for
 * other data rates may be configured to a value between 8 and 16 using the SFD_LENGTH register at sub
 * address 0x00
 * Standard SFD lengths: 6.8 Mbps: 8
 * 						 850 kbps: 16
 * 						 110 kbps: 64
 */

void set_sfd_improved(uint8_t datarate)
{
	uint8_t sfd_imp;

	switch (datarate)
	{
		case 0:														// 110 kbits/s
			sfd_imp = 0x40;
			break;
		case 1:														// 850 kbits/s
			sfd_imp = 0x10;
			break;
		case 2:														// 6.8 Mbits/s
			sfd_imp = 0x08;
			break;
	}

	DWM1000_spi_write_burst_reg(USR_SFD_ID, 0x00, &sfd_imp, 1);
}

/* AGC configuration and control:
 * AGC Tuning register 1 0x23:04. This is a 16 bit tuning register. This value needs to be changed depending on RXPRF.
 * AGC_TUNE1 values: 16 MHz PRF: 0x8870
 * 					 64 MHz PRF: 0x889B
 */


void set_agc_tune1(uint8_t prfindex)
{
	if (prfindex)
		DWM1000_write16bitoffsetregister(AGC_CFG_STS_ID, AGC_TUNE1_OFFSET, AGC_TUNE1_64M);
	else
		DWM1000_write16bitoffsetregister(AGC_CFG_STS_ID, AGC_TUNE1_OFFSET, AGC_TUNE1_16M);
}


/* Digital receiver configuration:
 * SFD detection timeout count 0x27:20. Set the 16-bit SFD detection timeout counter period, in units of preamble symbols.
 * The SFD detection timeout starts running as soon as preamble is detected.
 * By default this value is 4096+64+1 symbols, which is just longer the longest possible preamble and SFD sequence.
 * Never set this value to zero.
 */

void set_drx_tune3(uint16_t sfdTO)
{

	if(sfdTO == 0)													// Don't allow 0 - SFD timeout will always be enabled
	{
		sfdTO = DWT_SFDTOC_DEF;
	}

	DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_SFDTOC_OFFSET, sfdTO);
}


/* Digital receiver configuration:
 * Digital Tuning Register 2 0x27:08. This is a 32 bit tuning register. The value here needs to change depending on a number of parameters (PAC size, RXPRF config)
 * DRX_TUNE2 values: 	PAC size: 8  => RXPRF: - 16 MHz: 0x311A002D
 * 											   - 64 MHz: 0x313B006B
 * 						PAC size: 16 => RXPRF: - 16 MHz: 0x331A0052
 * 											   - 64 MHz: 0x333B00BE
 * 						PAC size: 32 => RXPRF: - 16 MHz: 0x351A009A
 * 											   - 64 MHz: 0x353B015E
 * 						PAC size: 64 => RXPRF: - 16 MHz: 0x371A011D
 * 											   - 64 MHz: 0x373B0296
 */

void set_drx_tune2(uint8_t prfindex, uint8_t rxpac)
{
	uint32_t drxtune2;

	if(prfindex)																		// 64 MHZ PRF
	{
		switch (rxpac)
		{
			case 0:
				drxtune2 = DRX_TUNE2_64MHZPRF_PAC8;
				break;
			case 1:
				drxtune2 = DRX_TUNE2_64MHZPRF_PAC16;
				break;
			case 2:
				drxtune2 = DRX_TUNE2_64MHZPRF_PAC32;
				break;
			case 3:
				drxtune2 = DRX_TUNE2_64MHZPRF_PAC64;
				break;
			default:
				break;
		}
	}
	else																				// 16 MHZ PRF
	{
		switch (rxpac)
		{
			case 0:
				drxtune2 = DRX_TUNE2_16MHZPRF_PAC8;
				break;
			case 1:
				drxtune2 = DRX_TUNE2_16MHZPRF_PAC16;
				break;
			case 2:
				drxtune2 = DRX_TUNE2_16MHZPRF_PAC32;
				break;
			case 3:
				drxtune2 = DRX_TUNE2_16MHZPRF_PAC64;
				break;
			default:
				break;
		}
	}

	DWM1000_write32bitoffsetregister(DRX_CONF_ID, DRX_TUNE2_OFFSET, drxtune2);
}


/* Digital receiver configuration:
 * 1. Digital Tuning Register 1b 0x27:06. This is a 16 bit tuning register. Values has to be change depending on use case (Preamble length, Data rate)
 * 		DRX_TUNE1b values: - Preamble lengths > 1024 symbols and 110 kbps operation: 						0x0064	      (Table 30 s.135 (User Manual))
 * 						   - Preamble lengths 128 to 1024 symbols for 850 kbps and 6.8 Mbps operation:	 	0x0020
 * 						   - Preamble length = 64 symbols, for 6.8 Mbps operation:							0x0010
 *
 * 2. Digital Tuning Register 0x27:26. This is a 16 bit tuning register. This value needs to change depending on the preamble length expected by the receiver
 * 		DRX_TUNE4H values: - Receive preamble length 64 symbols:					0x0010				(Table 32 s.138 (User Manual))
 * 						   - Receive preamble length 128 or greater symbols:		0x0028
 *
 */

void set_drx_tune1b_4H(uint8_t datarate, uint8_t txpreamlength)
{
	if(datarate == DWT_BR_110K)																		// Data rate 110 kbps
	{
		DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x0064);

		if(txpreamlength == DWT_PLEN_64)
		{
			DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_DRX_TUNE4HOFFSET, 0x0010);							// write DRX_TUNE4H: 0x0010
		}
		else
		{
			DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_DRX_TUNE4HOFFSET, 0x0028);							// write DRX_TUNE4H: 0x0028
		}
	}
	else
	{
		if(txpreamlength == DWT_PLEN_64)															// preamble length = 64 symbols and data rate 850 kbps or 6.8 Mbps
	    {
			DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x0010);				// write DRX_TUNE1b: 0x0010
			DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_DRX_TUNE4HOFFSET, 0x0010);							// write DRX_TUNE4H: 0x0010
	    }
	    else																						// preamble length 128 to 1024 symbols and data rate 850 kbps or 6.8 Mbps
	    {
	        DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x0020);				// write DRX_TUNE1b: 0x0020
	        DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_DRX_TUNE4HOFFSET, 0x0028);							// write DRX_TUNE4H: 0x0028
	    }
	}
}

/* Digital receiver configuration:
 * Digital Tuning Register 1a 0x27:02. This is a 16 bit tuning register.The value here needs to change depending on the RXPRF configuration
 * DRX_TUNE1a values: RXPRF configuration: 16 MHz PRF: 0x0087
 * 										   64 MHz PRF: 0x008D
 */

void set_drx_tune1a(uint8_t prfindex)
{
	if(prfindex)
	{
		DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_TUNE1a_OFFSET, DRX_TUNE1a_64MHZ_PRF); 	// 64 MHZ PRF
	}
	else
	{
		DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_TUNE1a_OFFSET, DRX_TUNE1a_16MHZ_PRF);		// 16 MHZ PRF
	}
}


/* Digital receiver configuration:
 * Digital Tuning Register 0b 0x27:02. This is a 16 bit tuning register. For optimal performance this value has to be change depending
 * on the adjusted "data rate" and on weather the SFD (start frame delimiter) sequence is configured to IEEE 802.15.4 UWB standard compliant SFD sequence, or the higher
 * performing non-standard SFD configuration.
 * DRX_TUNE0b values: 110 kbps:		Standard SFD: 			0x000A					(Table 28 s.134 (User Manual))
 * 									Non- Standard SFD: 		0x0016
 *
 * 					  850 kbps:		Standard SFD: 			0x0001
 * 									Non- Standard SFD: 		0x0006
 *
 * 					  6.8 Mbps:		Standard SFD: 			0x0001
 * 									Non- Standard SFD: 		0x0002
 */

void set_drx_tune0b(uint8_t datarate, uint8_t nssfd)
{
	uint16_t drx_tune0b;

	switch (datarate)
	{
		case 0:																			// Data rate: 110 kbps
			if(nssfd == 0)
				drx_tune0b = 0x000A;													// RX_SFTSH_LONG - standard
			else
				drx_tune0b = 0x0016;													// RX_SFTSH_USR_LONG - non-standard (DW - length specified above dwnsSFDlen)
			break;
		case 1:																			// Data rate: 850 kbps
			if(nssfd == 0)
				drx_tune0b = 0x0001;													// RX_SFTSH_LONG - standard
			else
				drx_tune0b = 0x0006;													// RX_SFTSH_USR_LONG - non-standard (DW - length specified above dwnsSFDlen)
			break;
		case 2:																			// Data rate: 6.8 kbps
			if(nssfd == 0)
				drx_tune0b = 0x0001;													// RX_SFTSH_LONG - standard
			else
				drx_tune0b = 0x0002;													// RX_SFTSH_USR_LONG - non-standard (DW - length specified above dwnsSFDlen)
			break;
		default:
			break;
	}

	DWM1000_write16bitoffsetregister(DRX_CONF_ID, DRX_TUNE0b_OFFSET, drx_tune0b);
}



/* Analog RF configuration:
 * Analog TX Control Register 0x28:0C. This is a 8 bit control register. It has to be set depending on the chosen TX Channel selected by TX_CHAN (register 0x1F).
 * Its also the bandwidth configuration to choose between Narrow band (Channel 1,2,3 or 5) and Wide band (Channel 4 or 7)
 * RF_TXCTRL values:  Channel 1: 	0x00005C40			(Table 35 s.141 (User Manual))
 * 					  Channel 2:	0x00045CA0
 * 					  Channel 3:	0x00086CC0
 * 					  Channel 4:	0x00045C80
 * 					  Channel 5:	0x001E3FE0
 * 					  Channel 7:	0x001E7DE0
 */
void set_tx_ctrl(uint8_t chan)
{
	uint32_t tx_config;

	switch (chan)
	{
		case 1:
			tx_config = RF_TXCTRL_CHAN1;
			break;
		case 2:
			tx_config = RF_TXCTRL_CHAN2;
			break;
		case 3:
			tx_config = RF_TXCTRL_CHAN3;
			break;
		case 4:
			tx_config = RF_TXCTRL_CHAN4;
			break;
		case 5:
			tx_config = RF_TXCTRL_CHAN5;
			break;
		case 7:
			tx_config = RF_TXCTRL_CHAN7;
			break;
		default:
			break;
	}

	DWM1000_write32bitoffsetregister (RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config);
}

/* Analog RF configuration:
 * Analog RX Control Register 0x28:0B. This is a 8 bit control register. It has to be set depending on the chosen RX Channel selected by RX_CHAN (register 0x1F).
 * Its also the bandwidth configuration to choose between Narrow band (Channel 1,2,3 or 5) and Wide band (Channel 4 or 7)
 * RF_RXCTRLH values: Channel 1,2,3,5: 	0xD8 		(Table 34 s.140 User Manual)
 * 					  Channel 4,7:		0xBC
 */

void set_rx_ctrl(uint8_t chan)
{
	uint8_t rx_config;

	if ((chan == 1) || (chan == 2) || (chan == 3) || (chan == 5))
		rx_config = RX_CTRL_CHAN_1_2_3_5;
	if ((chan == 4) || (chan == 7))
		rx_config = RX_CTRL_CHAN_4_7;

	DWM1000_spi_write_burst_reg(RF_CONF_ID, RF_RXCTRLH_OFFSET, &rx_config, 1);
}


/* Frequency synthesiser control:
 * PLL configuration 0x2B:07 and PLL Tuning 0x2B:0B. The values her has to be set depending on the chosen channel (depending on RX_CHAN and TX_CHAN (register: 0x1F)).
 * The config values are in table 40 p.148 (User Manual)
 * FS_PLLCFG values: Channel 1: 	0x09000407
 * 					 Channel 2,4: 	0x08400508
 * 					 Channel 3:		0x08401009
 * 					 Channel 5,7:	0x0800041D
 *
 *The tune values are in table 41 p.148 (User Manual)
 *FS_PLLTUNE values: Channel 1: 	0x1E
 * 					 Channel 2,4: 	0x26
 * 					 Channel 3:		0x56
 * 					 Channel 5,7:	0xBE	0xA6?
 */

void set_FS_PLL_config_tune(uint8_t chan)
{
	uint32_t chan_PLL;
	uint8_t chan_tune;
	switch (chan)
	{
		case 1:
			chan_PLL = PLL_CONFIG_CHAN_1;
			chan_tune = PLL_TUNE_CHAN_1;
			break;
		case 2:
			chan_PLL = PLL_CONFIG_CHAN_2_4;
			chan_tune = PLL_TUNE_CHAN_2_4;
			break;
		case 3:
			chan_PLL = PLL_CONFIG_CHAN_3;
			chan_tune = PLL_TUNE_CHAN_3;
			break;
		case 4:
			chan_PLL = PLL_CONFIG_CHAN_2_4;
			chan_tune = PLL_TUNE_CHAN_2_4;
			break;
		case 5:
			chan_PLL = PLL_CONFIG_CHAN_5_7;
			chan_tune = PLL_TUNE_CHAN_5_7;
			break;
		case 7:
			chan_PLL = PLL_CONFIG_CHAN_5_7;
			chan_tune = PLL_TUNE_CHAN_5_7;
			break;
		default:
			break;
	}

	DWM1000_spi_write_burst_reg(FS_CTRL_ID, FS_PLLTUNE_OFFSET, &chan_tune, 1);
	DWM1000_write32bitoffsetregister (FS_CTRL_ID, FS_PLLCFG_OFFSET, chan_PLL);
}

/* Leading Edge Detection Interface:
 * Set, configure the LDE algorithm parameters. The value of the LDE depends on the PRF (Pulse Repetition Frequency).
 * 1. Set the number of standard deviations to 13 (0xD) (more accuracy in close-up LOS conditions) or to 12 (0xC) (NLOS performance) => LDE Configuration Register 1 0x2E:0806 (LDE_CFG1)
 * 2. Set peak multiplier to value 3. This gives a factor of 1.5. => LDE Configuration Register 1 0x2E:0806 (LDE_CFG1)
 * 3. Set LDE configuration tuning register to the right value (0x2E:1806 CFG2). The value here needs to change depending on the RXPRF configuration
 * 	  For 16 MHz PRF: 0x1607 or for 64 MHz PRF: 0x0607
 */

void configure_lde(uint8_t prfindex)
{
	uint8_t lde_cfg1 = 0xD; //LDE_PARAM_CFG1;
	DWM1000_spi_write_burst_reg (LDE_IF_ID, LDE_CFG1_OFFSET, &lde_cfg1, 1);

	if(prfindex)																				// 16-bit LDE configuration tuning register
	{
		DWM1000_write16bitoffsetregister(LDE_IF_ID, LDE_CFG2_OFFSET, LDE_PARAM_CFG2_64); 		// 64 MHZ PRF
	}
	else
	{
	    DWM1000_write16bitoffsetregister(LDE_IF_ID, LDE_CFG2_OFFSET, LDE_PARAM_CFG2_16);		// 16 MHZ PRF
	}
}


uint16_t get_lde_replicaCoeff(uint16_t pcode)
{
	uint16_t ldecoff;

	switch (pcode)
	{
		case 1:
			ldecoff = LDE_REBLICA_COEFF_1;
			break;
		case 2:
			ldecoff = LDE_REBLICA_COEFF_2;
			break;
		case 3:
			ldecoff = LDE_REBLICA_COEFF_3;
			break;
		case 4:
			ldecoff = LDE_REBLICA_COEFF_4;
			break;
		case 5:
			ldecoff = LDE_REBLICA_COEFF_5;
			break;
		case 6:
			ldecoff = LDE_REBLICA_COEFF_6;
			break;
		case 7:
			ldecoff = LDE_REBLICA_COEFF_7;
			break;
		case 8:
			ldecoff = LDE_REBLICA_COEFF_8;
			break;
		case 9:
			ldecoff = LDE_REBLICA_COEFF_9;
			break;
		case 10:
			ldecoff = LDE_REBLICA_COEFF_10;
			break;
		case 11:
			ldecoff = LDE_REBLICA_COEFF_11;
			break;
		case 12:
			ldecoff = LDE_REBLICA_COEFF_12;
			break;
		case 13:
			ldecoff =  LDE_REBLICA_COEFF_13;
			break;
		case 14:
			ldecoff = LDE_REBLICA_COEFF_14;
			break;
		case 15:
			ldecoff = LDE_REBLICA_COEFF_15;
			break;
		case 16:
			ldecoff = LDE_REBLICA_COEFF_16;
			break;
		case 17:
			ldecoff = LDE_REBLICA_COEFF_17;
			break;
		case 18:
			ldecoff = LDE_REBLICA_COEFF_18;
			break;
		case 19:
			ldecoff = LDE_REBLICA_COEFF_19;
			break;
		case 20:
			ldecoff = LDE_REBLICA_COEFF_20;
			break;
		case 21:
			ldecoff = LDE_REBLICA_COEFF_21;
			break;
		case 22:
			ldecoff = LDE_REBLICA_COEFF_22;
			break;
		case 23:
			ldecoff = LDE_REBLICA_COEFF_23;
			break;
		case 24:
			ldecoff = LDE_REBLICA_COEFF_24;
			break;
		default:
			break;
	}
	return ldecoff;
}


uint32_t DWM1000_get_device_ID()
{
	uint32_t deviceID = 0;
	uint8_t rxdata[4] = {0,0,0,0};

	DWM1000_spi_read_burst_reg(DEV_ID_ID, 0x00, rxdata, 4);

	deviceID= *((uint32_t *) rxdata);

 return deviceID;
}


void DWM1000_spi_read_burst_reg(uint16_t address, uint16_t index, uint8_t * const rxbuffer, uint32_t count)
{
	uint8_t i = 0;
	uint8_t header[3];
	int numberofsub_index = 0;

// check if we need a sub-index or not: if BIT 6 of "index" is a 1 we need it, otherwise not

	if(index == 0)
	{
		header[numberofsub_index++] = (uint8_t) address; 			// if BIT 6 is 0: we have one header with only the register ID address
	}
	else
	{
		header[numberofsub_index++] = (uint8_t) (0x40 | address); 	// First Byte (Header): Bit 7 is READ, BIT6 is sub- index is present, BIT5-BIT0 is the register ID address

		if(index <= 0x7F)
		{
			header[numberofsub_index++] = (uint8_t) index;		 	// just a single sub- index byte. Second Byte: 8 bit Sub- address
		}
		else
		{
			header[numberofsub_index++] = 0x80 | (uint8_t) index;	// Second Byte: with 0x80 (BIT 7) = 1 we have an extended index, BIT6- BIT0 are the lower bits of the extended index (sub- address is longer than one byte)
			header[numberofsub_index++] = (uint8_t) (index >> 7);	// Third Byte: extended index: 8 bit long higher bits of the index
		}

	}

// SPI- read: read the number of bytes (count) out of the register (address)

	GPIO_PinOutClear(DWM1000_CS_PORT, DWM1000_CS_PIN);

	for (i=0; i < numberofsub_index; i++)							// First send the "Header" depended on the number of sub- indexes
	{
		USART_SpiTransfer(USART0, header[i]);
	}

	for (i=0; i < count; i++)										// Read the number of Bytes (count) to rxbuffer
		rxbuffer[i] = USART_SpiTransfer(USART0, 0x00);

	GPIO_PinOutSet(DWM1000_CS_PORT,DWM1000_CS_PIN);
}

void DWM1000_short_reset()
{
	// Set the RSTn pin as an output. Clear it to 0
	GPIO_PinModeSet(DWM1000_RSTn_PORT, DWM1000_RSTn_PIN, gpioModePushPull, false);
	//GPIO_PinOutClear(DWM1000_RSTn_PORT,DWM1000_RSTn_PIN);
	RTC_delay_ms(2);
	// Set the RSTn pin back to an input pin.
	GPIO_PinModeSet(DWM1000_RSTn_PORT, DWM1000_RSTn_PIN, gpioModeInput, false);
	RTC_delay_ms(10);

	DWM1000_idle_mode();
}

void DWM1000_soft_reset(void)
{
	uint8_t pmscctrl0[4];

	DWM1000_spi_read_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, 4);
	pmscctrl0[0] = 0x01;
	DWM1000_spi_write_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, 4);
	pmscctrl0[3] = 0x00;
	DWM1000_spi_write_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, 4);
	RTC_delay_ms(10);
	pmscctrl0[0] = 0x00;
	pmscctrl0[3] = 0xF0;
	DWM1000_spi_write_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, 4);

	DWM1000_idle_mode();
}

void DWM1000_spi_write_burst_reg(uint16_t address, uint16_t index, uint8_t * const txbuffer, uint32_t count)
{
	uint8_t i = 0;
	uint8_t header[3];
	int numberofsub_index = 0;

// check if we need a sub-index or not: if BIT 6 of "index" is a 1 we need it, otherwise not

	if(index == 0)
	{
		header[numberofsub_index++] = 0x80 | address; 				// BIT7 is 1: WRITE, if BIT 6 is 0: we have one header with only the register ID address
	}
	else
	{
		header[numberofsub_index++] = 0xC0 | address; 				// First Byte (Header): Bit 7 is WRITE, BIT6 is sub- index is present, BIT5-BIT0 is the register ID address

		if(index <= 0x7F)
		{
			header[numberofsub_index++] = (uint8_t) index;		 	// just a single sub- index byte. Second Byte: 8 bit Sub- address
		}
		else
		{
			header[numberofsub_index++] = 0x80 | (uint8_t) index;	// Second Byte: with 0x80 (BIT 7) = 1 we have an extended index, BIT6- BIT0 are the lower bits of the extended index (sub- address is longer than one byte)
			header[numberofsub_index++] = (uint8_t) (index >> 7);	// Third Byte: extended index: 8 bit long higher bits of the index
		}

	}

// SPI- write: read the number of bytes (count) out of the register (address)

	GPIO_PinOutClear(DWM1000_CS_PORT, DWM1000_CS_PIN);

	for (i=0; i < numberofsub_index; i++)							// First send the "Header" (register ID address + sub- addresses) depended on the number of sub- indexes
	{
		USART_SpiTransfer(USART0, header[i]);
	}

	for (i=0; i < count; i++)										// Write the values from the txbuffer to the choosen register address
	{
		 USART_SpiTransfer(USART0, txbuffer[i]);
	}

	GPIO_PinOutSet(DWM1000_CS_PORT,DWM1000_CS_PIN);
}

void DWM1000_write16bitoffsetregister(int address, int index, uint16_t value)
{
	uint8_t txbuffer[2];

	txbuffer[0] = value & 0xFF;
	txbuffer[1] = value >> 8;

	DWM1000_spi_write_burst_reg (address, index, txbuffer, 2);
}

void DWM1000_write32bitoffsetregister(int address, int index, uint32_t value)
{
	uint8_t txbuffer[4];
	int j;

	for (j=0;j<4;j++)
	{
		txbuffer[j] = value & 0xFF;
		value = value >> 8;
	}

	DWM1000_spi_write_burst_reg (address, index, txbuffer, 4);
}


uint16_t DWM1000_read16bitoffsetregister(int address, int index)
{
	uint8_t rxbuffer[2];
	uint16_t data;
	DWM1000_spi_read_burst_reg(address,index,rxbuffer,2);

	data = (rxbuffer[1] << 8) + rxbuffer[0];

	return data;
}

uint32_t DWM1000_read32bitoffsetregister(int address, int index)
{
	uint8_t rxbuffer[4];
	uint32_t data;

	DWM1000_spi_read_burst_reg(address,index,rxbuffer,4);
	data= *((uint32_t *) rxbuffer);

	return data;
}


void DWM1000_enable_clocks(uint8_t clock)
{
	uint8_t regdata[4];
	memset(regdata, 0, 4);

	DWM1000_spi_read_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, regdata, 4);

	switch (clock)
	{
		case ENABLE_ALL_SEQ:
		{
			DWM1000_spi_init(DWM1000_SPI_BAUDRATE_HIGH);
			regdata[0] = 0x00 ;
	        regdata[1] &= 0xFE;
	    }
	    break;
	    case FORCE_SYS_XTI:
	    {
	    	// System and RX
	    	DWM1000_spi_init(DWM1000_SPI_BAUDRATE_LOW);
	        regdata[0] &= 0xFC;
	        regdata[0] |= 0x01;
	    }
	    break;
	    case FORCE_SYS_PLL:
	    {
	        // System
	    	DWM1000_spi_init(DWM1000_SPI_BAUDRATE_HIGH);
	        regdata[0] &= 0xFC;
	        regdata[0] |= 0x02;
	    }
	    break;
	    case READ_ACC_ON:
	    {
	        regdata[0] = 0x48 | (regdata[0] & 0xB3);
	        regdata[1] = 0x80 | regdata[1];
	    }
	    break;
	    case READ_ACC_OFF:
	    {
	        regdata[0] = regdata[0] & 0xB3;
	        regdata[1] = 0x7F & regdata[1];
	    }
	    break;
	    case FORCE_OTP_ON:
	    {
	        regdata[1] = 0x02 | regdata[1];
	    }
	    break;
	    case FORCE_OTP_OFF:
	    {
	        regdata[1] = regdata[1] & 0xFD;
	    }
	    break;
	    case FORCE_TX_PLL:
	    {
	    	regdata[0] = 0x20 | (regdata[0] & 0xFC);
	    }
	    break;
	    default:
	    break;
	}
	    // Need to write lower byte separately before setting the higher byte(s)

		DWM1000_spi_write_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, &regdata[0], 1);
		DWM1000_spi_write_burst_reg(PMSC_ID, PMSC_CTRL0_OFFSET, regdata, 4);
}

uint32_t DWM1000_read_OTP(uint32_t address)
{
	uint8_t rxbuffer[4];
	uint32_t data;

	rxbuffer[0] = address & 0xff;											// write the low nibble of address to rx buffer
	rxbuffer[1] = (address>>8) & 0xff;										// write the high nibble of address to rx buffer

	DWM1000_spi_write_burst_reg(OTP_IF_ID, OTP_ADDR, rxbuffer,2);				// write the address to the OTP

	rxbuffer[0] = 0x03; 													// 0x03 for manual drive of OTP_READ
	DWM1000_spi_write_burst_reg(OTP_IF_ID, OTP_CTRL, rxbuffer,1);

	rxbuffer[0] = 0x01; 													// Bit0 is not autoclearing, so clear it (Bit 1 is but we clear it anyway).
	DWM1000_spi_write_burst_reg(OTP_IF_ID, OTP_CTRL, rxbuffer,1);

	data = DWM1000_read32bitoffsetregister (OTP_IF_ID, OTP_RDAT);			// Read read data, available 40ns after rising edge of OTP_READ

	rxbuffer[0] = 0x00; 													// Bit0 is not autoclearing, so clear it (Bit 1 is but we clear it anyway).
	DWM1000_spi_write_burst_reg(OTP_IF_ID, OTP_CTRL, rxbuffer,1);

	return data;															// Return the 32bit of read data
}

void DWM1000_loaducodefromrom(void)
{
	DWM1000_write32bitoffsetregister(PMSC_ID, PMSC_CTRL0_OFFSET, 0xF0300301);
	DWM1000_write16bitoffsetregister(OTP_IF_ID, OTP_CTRL, 0x8000);						// Set load LDE kick bit
	RTC_delay_ms(5);																	// Allow time for code to upload (should take up to 120 us)

	DWM1000_write32bitoffsetregister(PMSC_ID, PMSC_CTRL0_OFFSET, 0xF0300200);
	RTC_delay_ms(5);

	DWM1000_enable_clocks(ENABLE_ALL_SEQ);												// clock for sequencing, try locking PLL speed
	RTC_delay_ms(5);
}

/* Leading Edge Detection Interface:
 * LDE Receive Antenna Delay configuration 0x2E:1804. This is a 16 bit configuration register for setting the receive antenna delay.
 */

void DWM1000_set_rxantennadelay(uint16_t rxDelay)
{
	DWM1000_write16bitoffsetregister(LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay);
}

/* Transmitter Antenna Delay:
 * 16-bit Delay from Transmit to Antenna 0x18. This is a 16 bit configuration register for setting the transmit antenna delay.
 */

void DWM1000_set_txantennadelay(uint16_t txDelay)
{
	DWM1000_write16bitoffsetregister(TX_ANTD_ID, 0x00, txDelay);
}

/* Acknowledgement time and response time register 0x1A:
 * Is a configuration register used for specifying turn-around times for DW1000 to use when automatically switching between TX mode and RX modes.
 * Bits 0-19: W4R_TIM (Wait-for-Response turn-around Time: This 20-bit field is used to configure the turn-around time between TX complete and RX
 * enable when the wait for response function is being used. W4R_TIM is in units of approximately 1 µs, or 128 system clock cycles.
 * This is to save power by delaying the turn-on of the receiver, to align with the response time of the remote system, rather than turning on the
 * receiver immediately after transmission completes.)
 *
 * Bits 24-31: ACKTIM (Auto-Acknowledgement turn-around Time. This timer only applies if auto-acknowledgement is in use)
 */

void DWM1000_set_rxaftertxdelay(uint32_t rxDelayTime)
{
    uint32_t val = DWM1000_read32bitoffsetregister(ACK_RESP_T_ID, 0x00); 	// Read ACK_RESP_T_ID register

    val &= ~(ACK_RESP_T_W4R_TIM_MASK) ; 									// Clear the timer (19:0)

    val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK) ; 						// In UWB microseconds (e.g. turn the receiver on 20uus after TX)

    DWM1000_write32bitoffsetregister(ACK_RESP_T_ID, 0x00, val) ;
}

/* First read the last byte of system configuration register 0x04. This last byte contains the RXWTOE bit (Receive Wait Timeout Enable bit)
 * Second: When the time is >0 write the rx timeout time to Receive Frame Wait Timeout Period register 0x0C (16 bit config register)
 * 		   When the time is not >0 clear RXWTOE bit because the rx timeout time is not needed
 */

void DWM1000_set_rxtimeout(uint16_t time)
{
    uint8_t temp ;

    DWM1000_spi_read_burst_reg(SYS_CFG_ID,3, &temp, 1); 					// Read register 0x04 only the last 8 bit (Bits 24-31 (with 0x3))

    if(time > 0)
    {
        DWM1000_write16bitoffsetregister(RX_FWTO_ID, 0x0, time);

        temp |= (uint8_t)(SYS_CFG_RXWTOE>>24);								// set the RXWTOE bit (OR in 32bit value (1 bit set), I know this is in high byte)

        DWM1000local.sysCFGreg |= SYS_CFG_RXWTOE;							// Set RXWTOE bit also in the DWM1000local.sysCFGreg byte

        DWM1000_spi_write_burst_reg(SYS_CFG_ID,3, &temp, 1);
    }
    else
    {
        temp &= ~((uint8_t)(SYS_CFG_RXWTOE>>24));
        // AND in inverted 32bit value (1 bit clear), I know this is in high byte.
        DWM1000local.sysCFGreg &= ~(SYS_CFG_RXWTOE);

        DWM1000_spi_write_burst_reg(SYS_CFG_ID, 3, &temp, 1);

        //dwt_write16bitoffsetreg(RX_FWTO_ID,0,0) ; // Clearing the time is not needed
    }

}

/* Configuration of sleep and deepsleep mode:
 * AON Wakeup Configuration Register 0x2C:00. This is a 16 bit register to control what the DW1000 IC does as it wakes up from sleep or deepsleep
 * AON_WCFG values: 		DWT_PRESRV_SLEEP 0x0100	=> preserve sleep bit set to 1: these sleep controls are not cleared upon wake up
 *      					DWT_LOADOPSET    0x0080 => load operating parameter set on wakeup
 *      					DWT_CONFIG       0x0040 => (ONW_LDC) download the AON array into the HIF (configuration download)
 *      					DWT_LOADEUI      0x0008	=> load the EUI from OTP in reg 0x01
 *      					DWT_GOTORX       0x0002 => On wake up turn on receiver => automatically in RX
 *      					DWT_TANDV        0x0001
 *
 * AON Configuration Register 0 register 0x2C:06. This is a 32 bit register for AON. Configuration of wake up are the lowest 8 bit.
 * AON_CFG0 values: 		DWT_XTAL_EN      0x10 - keep XTAL running during sleep
 *      					DWT_WAKE_SLPCNT  0x8 - wake up after sleep count
 *      					DWT_WAKE_CS      0x4 - wake up on SPICSn
 *      					DWT_WAKE_WK      0x2 - wake up on WAKEUP PIN
 *      					DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 *
 *
 */

void DWM1000_config_sleep_mode(uint16_t mode, uint8_t wakeup)
{
	mode |= DWM1000local.sleep_mode;										// ADD the ONW_LLDE and ONW_LLDO value from init to the mode
	DWM1000_write16bitoffsetregister(AON_ID, AON_WCFG_OFFSET, mode);

	DWM1000_spi_write_burst_reg(AON_ID, AON_CFG0_OFFSET, &wakeup, 1);		// write the wake up config to AON_CFG0

}


void DWM1000_enter_sleepmode(void)
{
	DWM1000_AON_arrayupload();
}

void DWM1000_AON_arrayupload(void)
{
	 uint8_t buffer;

	 buffer = 0x00;
	 DWM1000_spi_write_burst_reg(AON_ID, AON_CTRL_OFFSET, &buffer, 1);

	 buffer = 0x02;
	 DWM1000_spi_write_burst_reg(AON_ID, AON_CTRL_OFFSET, &buffer, 1);
}

void DWM1000_SPI_Wake_Up(uint8_t *wakeupbuffer, uint16_t length)
{
	DWM1000_spi_init(DWM1000_SPI_BAUDRATE_LOW);
	DWM1000_spi_read_burst_reg(0x0, 0x0, wakeupbuffer, length);

	//RTC_delay_ms(5);
}


void DWM1000_send_poll_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length)
{
	DWM1000_new_transmit();

	// set Destination ID and Source ID for poll message
	DWM1000_set_message_array_bytes(&tx_message[DESTINATION_ID_POSITION], DESTINATION_ID, 2);
	DWM1000_set_message_array_bytes(&tx_message[SOURCE_ID_POSITION], SOURCE_ID, 2);

	DWM1000_prepare_tx_message(tx_message, message_length, 0);										// write the message to the tx buffer

	//Todo error check, maybe do it again
	if(!(DWM1000_start_transmit(DWT_START_TX_IMMEDIATE, DWT_NO_RESPONSE)))								// transmit the data immediate and with aresponse expected
	{
		/* Poll DW1000 until TX frame sent event set.*/
		while (!(DWM1000_read32bitoffsetregister(SYS_STATUS_ID, 0) & SYS_STATUS_TXFRS));
	}

	DWM1000local.sysctrl = 0;
	DWM1000_start_Receive();
	DWM1000_deviceMode = STATE_RECEIVE;
}


int DWM1000_send_response_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length)
{
	int errorcheck= DWT_ERROR;
	uint64_t tx_response_timestamp = 0;

	DWM1000_new_transmit();

	uint64_t resp_tx_time = (POLL_RX_TO_RESP_TX_DLY_UUS * TIME_RES_INV);
	tx_response_timestamp = DWM1000_set_delayed_txrx_time(resp_tx_time);

	// set Destination ID and Source ID for poll message
	DWM1000_set_message_array_bytes(&tx_message[DESTINATION_ID_POSITION], DESTINATION_ID, 2);
	DWM1000_set_message_array_bytes(&tx_message[SOURCE_ID_POSITION], SOURCE_ID, 2);

	DWM1000_prepare_tx_message(tx_message, message_length, 0);										// write the message to the tx buffer

	//Todo error check, maybe do it again
	if(!(DWM1000_start_transmit(DWT_START_TX_DELAYED, DWT_NO_RESPONSE)))							// transmit the data delayed
	{
		/* Poll DW1000 until TX frame sent event set.*/
		while (!(DWM1000_read32bitoffsetregister(SYS_STATUS_ID, 0) & SYS_STATUS_TXFRS));

		errorcheck= DWT_SUCCESS;
	}

	DWM1000local.sysctrl = 0;
	DWM1000_start_Receive();
	DWM1000_deviceMode = STATE_RECEIVE;

	return errorcheck;
}


int DWM1000_send_final_message(uint8_t *tx_message, uint32_t DESTINATION_ID, uint32_t SOURCE_ID, uint8_t message_length, uint64_t poll_tx_timestamp, uint64_t resp_rx_timestamp)
{
	int errorcheck= DWT_ERROR;
	uint64_t final_tx_timestamp = 0;

	DWM1000_new_transmit();

	uint64_t final_tx_time = (RESP_RX_TO_FINAL_TX_DLY_UUS * TIME_RES_INV);								// 4430 us
	final_tx_timestamp = DWM1000_set_delayed_txrx_time(final_tx_time);

	// set Destination ID and Source ID for poll message
	DWM1000_set_message_array_bytes(&FINAL_message[DESTINATION_ID_POSITION], DESTINATION_ID, 2);
	DWM1000_set_message_array_bytes(&FINAL_message[SOURCE_ID_POSITION], SOURCE_ID, 2);

	DWM1000_set_message_array_bytes(&tx_message[POLL_TX_TIMESTAMP_POSITION], poll_tx_timestamp, 5);
	DWM1000_set_message_array_bytes(&tx_message[RESP_RX_TIMESTAMP_POSITION], resp_rx_timestamp, 5);
	DWM1000_set_message_array_bytes(&tx_message[FINAL_TX_TIMESTAMP_POSITION], final_tx_timestamp, 5);

	DWM1000_prepare_tx_message(tx_message, message_length, 0);											// write the message to the tx buffer

	//Todo error check, maybe do it again
	if(!(DWM1000_start_transmit(DWT_START_TX_DELAYED, DWT_NO_RESPONSE)))								// transmit the data delayed
	{
		/* Poll DW1000 until TX frame sent event set.*/
		while (!(DWM1000_read32bitoffsetregister(SYS_STATUS_ID, 0) & SYS_STATUS_TXFRS));

		/* Clear TXFRS event. */
		//DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, SYS_STATUS_TXFRS);
		errorcheck= DWT_SUCCESS;
	}

	DWM1000local.sysctrl = 0;
	DWM1000_start_Receive();
	DWM1000_deviceMode = STATE_RECEIVE;

	return errorcheck;
}


/* Function to write bytes into an array. This written byte code can be 64 bit wide, so you can directly write the rx/tx timestamp
 * to the Message array at the right position.
 */

void DWM1000_set_message_array_bytes(uint8_t *buffer, uint64_t writebytes, uint8_t numberofbytes)
{
	int i=0;
	for (i=0; i < numberofbytes; i++)
	{
		buffer[i]= (uint8_t) writebytes;
		writebytes >>= 8;
	}
}

void DWM1000_get_message_array_bytes(uint8_t *buffer, uint64_t *readbytes, uint8_t numberofbytes)
{
	int64_t i=0;
	*readbytes=0;
	for (i=0; i < numberofbytes; i++)
	{
		uint64_t temp = buffer[i];
		*readbytes |= temp << (i * 8);
	}
}


/* Write poll message to TX Buffer of the DW1000
 *
 */

void DWM1000_prepare_tx_message(uint8_t *tx_packet, uint8_t packet_length, uint16_t txbufferoffset)
{
	DWM1000_spi_write_burst_reg (TX_BUFFER_ID, 0, tx_packet, packet_length-2);						// Write the data to the IC TX buffer, (-2 bytes for auto generated CRC => packetlength -2)

	uint32_t msg_length = DWM1000local.txFCTRL | packet_length | (txbufferoffset << 22);
	DWM1000_write32bitoffsetregister(TX_FCTRL_ID, 0x00, msg_length);
}


/* Before starting the transmit there are 4 possible modes:
 *
 * 1. immediate transmission with no response expected 	(DWT_START_TX_IMMEDIATE, DWT_NO_RESPONSE)
 * 2. delayed transmission with no response expected	(DWT_START_TX_DELAYED, DWT_NO_RESPONSE)
 * 3. immediate transmission with response expected, so the receiver will be automatically turned on after TX is done 	(DWT_START_TX_IMMEDIATE, DWT_RESPONSE_EXPECTED)
 * 4. delayed transmission with response expected, so the receiver will be automatically turned on after TX is done 	(DWT_START_TX_DELAYED, DWT_RESPONSE_EXPECTED)
 *
 * Response expected: Set the wait for response bit (WAIT4RESP bit) in SYS_CTRL register: 0x0D
 * Delayed transmit:  Set the transmit start bit (TXSTRT) and the transmitter delayed sending bit (TXDLYS) at the same time in SYS_CTRL register: 0x0D
 * 					  Error check: if Half Period Delay Warning bit (HDPWARN) and/or Transmit power up time error bit (TXPUTE) was set.
 * 							With an error: abort the delayed TX/RX by issuing a TRXOFF transceiver off command (TRXOFF bit, reg: 0x0D), give back that we had an transmission error
 * 							Without error: give back that the transmission was successful
 *
 * Immediate transmit: Set only the transmit start bit (TXSTRT)
 *
 */

int DWM1000_start_transmit(uint8_t txmode, uint8_t rxmode)
{

	uint8_t txbyte= 0;
	uint16_t checkTx= 0;
	int errorcheck= DWT_SUCCESS;

	switch (txmode)
	{
		case DWT_START_TX_IMMEDIATE:
		{
			if(rxmode == DWT_RESPONSE_EXPECTED)
			{
				txbyte = (uint8_t)(SYS_CTRL_WAIT4RESP | SYS_CTRL_TXSTRT);							// set the wait for response bit and the start bit
				DWM1000_spi_write_burst_reg(SYS_CTRL_ID, 0, &txbyte,1);
				DWM1000local.wait4resp = 1;
			}
			else
			{
				txbyte |= (uint8_t) SYS_CTRL_TXSTRT;												// set TXSTRT
				DWM1000_spi_write_burst_reg(SYS_CTRL_ID, 0, &txbyte,1);
			}
		}
		break;
		case DWT_START_TX_DELAYED:																	// delayed transmission
		{
			if (rxmode == DWT_RESPONSE_EXPECTED)
			{
				txbyte |= (uint8_t)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT | SYS_CTRL_WAIT4RESP);		// set TXDLYS and TXSTRT at the same time
				DWM1000_spi_write_burst_reg(SYS_CTRL_ID, 0, &txbyte,1);
			}
			else
			{
				txbyte |= (uint8_t)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);								// set TXDLYS and TXSTRT at the same time
				DWM1000_spi_write_burst_reg(SYS_CTRL_ID, 0, &txbyte,1);
			}
			checkTx= DWM1000_read16bitoffsetregister(SYS_STATUS_ID,3);								// read out the control bits HDPWARN an TXPUTE bits

			if ((checkTx & SYS_STATUS_TXERR) == 0)
			{
				errorcheck= DWT_SUCCESS;
			}
			else
			{
				// the TXDLYS was set too late for the specified DX_TIME
				txbyte= (uint8_t) SYS_CTRL_TRXOFF;													// abort the delayed TX/RX by issuing a TRXOFF transceiver off command
				DWM1000_spi_write_burst_reg(SYS_CTRL_ID, 0, &txbyte, 1);

				DWM1000local.wait4resp = 0;
				errorcheck= DWT_ERROR;
			}
		}
		break;
		default:
		break;

	}

	return errorcheck;
}


/* Turn on the receiver immediate or delayed
 * Receiver stayed turned on until it receives a good frame, an error (CRC, PHY header...) or it times out (SFD, Preamble or frame)
 *
 * Two options:	1. immediate: receiver turned on immediate (delayed = 0). Set the RXENAB in SYS_CTRL register 0x0D
 * 				2. delayed: receiver turned on delayed, that means that it turns on after some delay (delayed = 1)
 * 				   			First set the RXENAB bit and the delay bit RXDLYE in SYS_CTRL register 0x0D
 * 				   			Secondly check for errors: read the SYS_STATUS register	0x0F, then check the HPDWARN bit
 *													   Is there an error then turn the delayed receive off and clear the delay bit
 *
 *
 */

int DWM1000_enable_rx(int delayed)
{
	uint16_t rxtemp ;
	uint8_t temp1 = 0;

	//DWM1000_sync_rxbuffer_pointers();

	rxtemp = (uint16_t) SYS_CTRL_RXENAB ;

	if (delayed)
	{
		rxtemp |= (uint16_t) SYS_CTRL_RXDLYE ;
	}

	DWM1000_write16bitoffsetregister(SYS_CTRL_ID,0,rxtemp) ;

	if (delayed) 																// Check for errors
	{
		DWM1000_spi_read_burst_reg(SYS_STATUS_ID, 3, &temp1, 1) ;				// read out the SYS_STATUS register third byte

	    if (temp1 & (SYS_STATUS_HPDWARN >> 24)) 								// If delay has not passed do delayed else immediate RX on
	    {
	    	// todo this function
	    	DWM1000_transceiver_off();											// Turn the delayed receive off, and do immediate receive, return warning indication with DWT_ERROR
	        rxtemp = (uint16_t) SYS_CTRL_RXENAB; 								// Clear the delay bit
	        DWM1000_write16bitoffsetregister(SYS_CTRL_ID,0,rxtemp) ;
	        return DWT_ERROR;
	    }
	}

	return DWT_SUCCESS;
}


/*	Read out the received message
 */


void DWM1000_get_receive_message(uint8_t *rx_msg)
{
	uint32_t frame_length;

	DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, DWM1000local.sysstatus | SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_LDEERR | SYS_STATUS_RXRFSL);	// Clear the RX Status flags

	frame_length = DWM1000_read32bitoffsetregister(RX_FINFO_ID, 0) & RX_FINFO_RXFLEN_MASK;				// A frame has been received, read it into the local buffer
	if (frame_length <= RX_BUFFER_LEN)
	{
		DWM1000_read_receive_data(rx_msg, frame_length, 0);
	}
}


void DWM1000_read_receive_data(uint8_t *tx_frame, uint16_t length, uint16_t bufferoffset)
{
	DWM1000_spi_read_burst_reg(RX_BUFFER_ID, bufferoffset, tx_frame, length);
}


int DWM1000_check_receive_message(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE)
{
	rx_buffer[SEQU_NUMB_POS]=0;

	// check the Frame contol byte and the PAN ID
	if (!(memcmp(rx_buffer, POLL_message, 5) == 0))
	{
		return DWT_ERROR;
	}

	// Read source address
	uint32_t source_address = (((uint32_t) rx_buffer[8]) << 8 | rx_buffer[7]);
	if (*SOURCE_ID == 0x00)
	{  																						// If Source is empty, return source address
		*SOURCE_ID = source_address;
	}
	else																					// If Source is NOT empty, compare it
	{
		if (source_address != *SOURCE_ID)
		{
			return DWT_ERROR;
		}
	}

	// Read destination address
	uint32_t destination_addr = (((uint32_t) rx_buffer[6]) << 8 | rx_buffer[5]);
	if (*DESTIN_ID == 0x00) 																// If Destination is empty, return Destination address
	{
		*DESTIN_ID = destination_addr;
	}
	else
	{
		if (destination_addr != *DESTIN_ID)
		{
			return DWT_ERROR;
		}
	}

	// Check Message Type
	uint8_t message_type = rx_buffer[9];
	if (*MSG_TYPE == 0x00) 												// If Message type is empty, return message_type
	{
		*MSG_TYPE = message_type;
	}
	else
	{
		if (message_type != *MSG_TYPE)
		{
			return DWT_ERROR;
		}
	}

	//todo check activity parameter

	return DWT_SUCCESS;
}



/* Function to synchronize the rx buffer pointers. The host and the IC buffer pointers has to be aligned before starting RX
 * 1. Read the SYS_STATUS register 0x0F: read the third byte of the register (bits 24-31) to get the ICRBP and HSRBP bit values
 * 2. When this bits are different: set the HRBPT bit (Host Side Receive Buffer Pointer Toggle) in SYS_CTRL register 0x0D to toggle the buffer.
 */

void DWM1000_sync_rxbuffer_pointers(void)
{
	uint8_t buffer;

	DWM1000_spi_read_burst_reg(SYS_STATUS_ID, 3, &buffer, 1);

	if((buffer & (SYS_STATUS_ICRBP >> 24)) != ((buffer & (SYS_STATUS_HSRBP>>24)) << 1))     	// IC side Receive Buffer Pointer	   																						 	// Host Side Receive Buffer Pointer
	{
		uint8_t hrbpt = 0x01;
	    DWM1000_spi_write_burst_reg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, &hrbpt, 1) ;				// We need to swap RX buffer status reg (write one to toggle internally)
	}
}


/* Turn the Transceiver off:
 * First read the complete Event Mask Register 0x0E. Thats because, before disable the transceiver a interrupt can be set.
 * => disable interrupts for this operation (uC)
 * Clear the interrupt mask register
 * Disable the radio transceiver
 *
 */


void DWM1000_transceiver_off()
{
	//int dwm_IRQ_stat;
	uint8_t temp ;
	uint32_t irq_mask;

	temp = (uint8_t) SYS_CTRL_TRXOFF ; 												// TRXOFF in the lowest byte of SYS_CTRL

	irq_mask = DWM1000_read32bitoffsetregister(SYS_MASK_ID,0); 						// Read the System Event Mask Register (Interrupt)

	//TODO interrupts ausschalten und status auslesen
	//dwm_IRQ_stat = decamutexon();

	DWM1000_write32bitoffsetregister(SYS_MASK_ID, 0, 0) ; 							// Clear interrupt mask in SYS_MASK register

	DWM1000_spi_write_burst_reg(SYS_CTRL_ID, 0, &temp, 1) ; 						// Disable the radio

	// Forcing Transceiver off - so we do not want to see any new events that may have happened
	DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD));

	DWM1000_sync_rxbuffer_pointers();

	DWM1000_write32bitoffsetregister(SYS_MASK_ID, 0, irq_mask) ; 					// Set the interrupt mask back to original

	// TODO Enable/restore interrupts again...
	// decamutexoff(dwm_IRQ_stat) ;
	DWM1000local.wait4resp = 0;
}


uint64_t DWM1000_get_rx_timestamp_40bit(void)
{
	uint8_t rx_timestamp_buffer[8];
	uint64_t rx_timestamp = 0;

	memset(rx_timestamp_buffer, 0, 8);
	DWM1000_read_rx_timestamp_40bit(rx_timestamp_buffer);

	rx_timestamp = *((uint64_t *)rx_timestamp_buffer);

	DWM1000_correct_timestamp(&rx_timestamp);

	return rx_timestamp;
}

void DWM1000_read_rx_timestamp_40bit(uint8_t *rx_timestamp)
{
	DWM1000_spi_read_burst_reg(RX_TIME_ID, 0, rx_timestamp, RX_TIME_RX_STAMP_LEN);
}


uint64_t DWM1000_get_tx_timestamp_40bit(void)
{
	uint8_t tx_timestamp_buffer[8];
	uint64_t tx_timestamp=0;

	memset(tx_timestamp_buffer, 0, 8);
	DWM1000_read_tx_timestamp_40bit(tx_timestamp_buffer);

	tx_timestamp = *((uint64_t *)tx_timestamp_buffer);

	return tx_timestamp;
}

void DWM1000_read_tx_timestamp_40bit(uint8_t *tx_timestamp)
{
	DWM1000_spi_read_burst_reg(TX_TIME_ID, 0, tx_timestamp, TX_TIME_TX_STAMP_LEN);
}


uint64_t DWM1000_set_delayed_txrx_time(uint64_t start_time)
{
	uint8_t delayBytes[8];
	uint8_t systemtimebytes[8];
	uint64_t timestamp = 0;

	memset(systemtimebytes, 0, 8);
	DWM1000_spi_read_burst_reg(SYS_TIME_ID, 0, systemtimebytes, 5);										// get system time

	timestamp = *((uint64_t *)systemtimebytes);

	timestamp += start_time;

	memset(delayBytes, 0, 8);
	for(uint8_t i=0; i<5; i++)
	{
		delayBytes[i] = (uint8_t)((timestamp >> (i*0x8)) & 0xFF);
	}
	delayBytes[0] = 0;
	delayBytes[1] &= 0xFE;

	DWM1000_spi_write_burst_reg(DX_TIME_ID, 0, delayBytes, 5);

	timestamp = *((uint64_t *)delayBytes);

	return (timestamp += TX_ANT_DLY);
}

uint32_t DWM1000_get_tx_timestamp_32bit(void)
{
	return DWM1000_read32bitoffsetregister(TX_TIME_ID, 0);
}

uint32_t DWM1000_get_rx_timestamp_32bit(void)
{
	return DWM1000_read32bitoffsetregister(RX_TIME_ID, 0);
}


void DWM1000_set_irq_mask_register(uint32_t irq_mask)
{
	DWM1000_write32bitoffsetregister(SYS_MASK_ID, 0x00, irq_mask);
}

void DWM1000_clear_irq_mask_register(void)
{
	DWM1000_write32bitoffsetregister(SYS_MASK_ID, 0x00, 0x0000);
}

enum states_tag DWM1000_receiver_on_state(void)
{
	DWM1000_new_receive();
	DWM1000_permanent_receive(true);

	DWM1000_set_rxtimeout(0);
	DWM1000_enable_rx(0);

	return STATE_RECEIVE_MODE;
}


void DWM1000_idle_mode(void)
{
	DWM1000local.sysctrl = 0x0000;															// set all 4 bytes of variable sysctrl to zero
	DWM1000local.sysctrl |= SYS_CTRL_TRXOFF;												// Tranceiver OFF => IDLE MODE
	DWM1000_write32bitoffsetregister(SYS_CTRL_ID, 0x00 , DWM1000local.sysctrl);
	DWM1000_deviceMode = STATE_IDLE;
}


void DWM1000_clear_receive_status_bits(void)
{
	DWM1000local.sysstatus = 0x0000;														// set all 4 bytes of variable sysstatus to zero

	DWM1000local.sysstatus = (SYS_STATUS_RXDFR | SYS_STATUS_LDEDONE | SYS_STATUS_LDEERR | SYS_STATUS_RXPHE |
							   SYS_STATUS_RXFCE | SYS_STATUS_RXFCG | SYS_STATUS_RXRFSL);
	DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0x00, DWM1000local.sysstatus);
}

void DWM1000_new_receive()
{
	DWM1000_idle_mode();
	DWM1000local.sysctrl = 0x0000;														// set all 4 bytes of variable sysctrl to zero
	DWM1000_clear_receive_status_bits();
	DWM1000_deviceMode = STATE_RECEIVE;
}

void DWM1000_permanent_receive(bool val)
{
	if(val)
	{
		DWM1000local.sysCFGreg |= SYS_CFG_RXAUTR;
		DWM1000_write32bitoffsetregister(SYS_CFG_ID, 0x00, DWM1000local.sysCFGreg);
	}
}

void DWM1000_clear_transmit_status_bits(void)
{
	DWM1000local.sysstatus = 0x0000;													// set all 4 bytes of variable sysstatus to zero

	DWM1000local.sysstatus = (SYS_STATUS_TXFRB | SYS_STATUS_TXPRS | SYS_STATUS_TXPHS | SYS_STATUS_TXFRS);
	DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0x00, DWM1000local.sysstatus);
}


void DWM1000_new_transmit(void)
{
	DWM1000_idle_mode();
	DWM1000local.sysctrl = 0x0000;														// set all 4 bytes of variable sysctrl to zero
	DWM1000_clear_transmit_status_bits();
	DWM1000_deviceMode = STATE_TRANSMIT;
}

void DWM1000_setDefaults(void)
{
	if (DWM1000_deviceMode == STATE_TRANSMIT)
	{

	}
	else if (DWM1000_deviceMode == STATE_RECEIVE)
	{

	}
	else if (DWM1000_deviceMode == STATE_IDLE)
	{
		DWM1000_set_irq_mask_register(SYS_MASK_MASK_32 & (SYS_MASK_MRXFCG | SYS_MASK_MRXFCE | SYS_MASK_MLDEERR | SYS_MASK_MRXPHE | SYS_MASK_MRXPHE | SYS_MASK_MRXRFSL));// | SYS_MASK_MHPDWARN));				// set MRXFCG bit (14) and MRXFCE (15) at system event mask register to enable the interrupt of the DWM for a good received event (FCS) or a error error event (FCE)
		GPIO_IntConfig(DWM1000_IRQ_PORT, DWM1000_IRQ_PIN, true, false, true);

		DWM1000_permanent_receive(true);
	}
}

void DWM1000_configuration2(volatile dwt_config_t *config)
{
	uint8_t nsSfd_result  = 0;
	uint8_t useDWnsSFD = 0;
	uint32_t regval;
	uint16_t lde_coeff = get_lde_replicaCoeff(config->rxCode);
	uint8_t prfIndex = DWM1000local.prfIndex = config->prf - DWT_PRF_16M;
	uint8_t chan = DWM1000local.chan = config->chan;
	uint16_t sfdTO = config->sfdTO;
	uint8_t smarttxpower = config -> smarttxpower;

	/* Set the data rate (Mbps) (110 kbps, 850 kbps, 6.8 Mbps):
	 * - set it to 110 kbps we have to set RXM110K bit in 0x04 register (System Configuration) to 1 otherwise clear this bit to 0
	 * - set also the LDE Replica Coeffiecient depending on RX_PCODE (here config->rxcode) (depends on channel number and PRF settings see s. 203 table 58)
	 * 		- for 110 kbps divide the replica coeffiecent by 8
	 * - set the PHR mode: Set PHR_MODE bits (17,16) in 0x04 register (System Configuration)
	 * 		- PHR_Mode: 00: Standard Frame mode. Use this setting is for IEEE 802.15.4 compliance
	 * 					11: Long Frames mode. Proprietary PHR encoding. Frame Length 0-1023. not IEEE standard (not allowed)
	 */

	// Set (non-standard) user SFD for improved performance
	if(config->nsSFD)
	{
		set_sfd_improved(config->dataRate);												// Write non standard (DW) SFD length
		nsSfd_result = 3 ;
		useDWnsSFD = 1 ;
	}

	DWM1000_write32bitoffsetregister(PANADR_ID, 0, 0x000A0001);							// write PAN ID

	if(DWT_BR_110K == config->dataRate)
	{
		DWM1000local.sysCFGreg |= SYS_CFG_RXM110K;										// set RXM110K to 1 and safe it in DWM1000local.sysCFGreg
		   lde_coeff >>= 3; 															// lde_replicaCoeff must be divided by 8
	}
	else
	{
		DWM1000local.sysCFGreg &= ~(SYS_CFG_RXM110K);									// keep RXM110K bit low to get 850 kbps or 6.8 Mbps
	}
	if(!smarttxpower)
	{
		DWM1000local.sysCFGreg |= SYS_CFG_DIS_STXP;										// Disable Smart TX Power
	}
	else
	{
		DWM1000local.sysCFGreg &= ~(SYS_CFG_DIS_STXP);									// Enable Smart TX Power
	}

	DWM1000local.longFrames = config->phrMode ;
	DWM1000local.sysCFGreg |= (SYS_CFG_PHR_MODE_11 & (config->phrMode << 16));			// depending on config->phrMode settings bit masking of PHR_MODE bits

	DWM1000local.sysCFGreg |= SYS_CFG_RXAUTR;											// set permanent RECEIVE bit

	DWM1000_write32bitoffsetregister(SYS_CFG_ID, 0x00, DWM1000local.sysCFGreg);			// set PHR-Mode in 0x04 register, set the hole 0x04 register

	DWM1000_set_irq_mask_register(SYS_MASK_MASK_32 & (SYS_MASK_MRXFCG | SYS_MASK_MRXFCE | SYS_MASK_MLDEERR | SYS_MASK_MRXPHE | SYS_MASK_MRXPHE | SYS_MASK_MRXRFSL));// | SYS_MASK_MHPDWARN));				// set MRXFCG bit (14) and MRXFCE (15) at system event mask register to enable the interrupt of the DWM for a good received event (FCS) or a error error event (FCE)
	GPIO_IntConfig(DWM1000_IRQ_PORT, DWM1000_IRQ_PIN, true, false, true);

	// Configure AGC parameters
	set_agc_tune1 (prfIndex);															// AGC Tuning register 1
	DWM1000_write32bitoffsetregister(AGC_CFG_STS_ID, AGC_TUNE2_OFFSET, AGC_TUNE2_VAL);	// AGC Tuning register 2 0x23:0C. write value 0x2502A907 (s.115 User Manual)
	DWM1000_write16bitoffsetregister(AGC_CFG_STS_ID, AGC_TUNE3_OFFSET, AGC_TUNE3_VAL);  // AGC Tuning register 3 0x23:12. write value 0x0035 (s.115 User Manual)

	// Configure the baseband parameters
	set_drx_tune0b(config->dataRate, config->nsSFD);									// Configure the baseband parameter DTUNE0 (for specified data rate and SFD settings)
	set_drx_tune1a(prfIndex);															// Configure the baseband parameter DTUNE1a (register 0x27:04) (for specified PRF)
	set_drx_tune1b_4H(config->dataRate, config->txPreambLength);
	set_drx_tune2(prfIndex, config->rxPAC);

	configure_lde(prfIndex);															// 16-bit LDE configuration tuning register

	DWM1000_write16bitoffsetregister(LDE_IF_ID, LDE_REPC_OFFSET, lde_coeff);			// Set the LDE Replica Coeffiecient in 0x2E register

	set_TX_Power_config(prfIndex, chan, smarttxpower);									// set tx power depended on smartpowerbit

	set_rx_ctrl(chan);																	// Configure RF RX blocks (for specified channel/bandwidth)

	set_tx_ctrl(chan);																	// Configure RF TX blocks (for specified channel and PRF)

	set_TC_PGDELAY(chan);																// Configure Pulse Generator Delay value

	set_FS_PLL_config_tune(chan);														// Configure frequency synthesiser config and tune

	regval = (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | 						// Transmit Channel
		     (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | 						// Receive Channel
		     (CHAN_CTRL_RXFPRF_MASK & (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | 					// RX PRF
		     ((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | 	// nsSFD enable RX&TX
		     (CHAN_CTRL_DWSFD & (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | 							// Use DW nsSFD
		     (CHAN_CTRL_TX_PCOD_MASK & (config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | 				// TX Preamble Code
		     (CHAN_CTRL_RX_PCOD_MASK & (config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)) ; 				// RX Preamble Code

	DWM1000_write32bitoffsetregister(CHAN_CTRL_ID,0,regval);

	// Set up TX Preamble Size and TX PRF
	// Set up TX Ranging Bit and Data Rate
	DWM1000local.txFCTRL = (config->txPreambLength | config->prf) << 16;
	DWM1000local.txFCTRL |= (config->dataRate << TX_FCTRL_TXBR_SHFT) | TX_FCTRL_TR; 				// Always set ranging bit !!!
	DWM1000_write32bitoffsetregister(TX_FCTRL_ID,0,DWM1000local.txFCTRL);

	DWM1000_set_rxantennadelay(RX_ANT_DLY);
	DWM1000_set_txantennadelay(TX_ANT_DLY);

	//todo: EUI ADD read

	DWM1000local.panID = DWM1000_read32bitoffsetregister(PANADR_ID,0);

	DWM1000_idle_mode();

	DWM1000local.sysstatus = SYS_STATUS_LDEDONE | SYS_STATUS_RXPHE | SYS_STATUS_RXDFR | SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_LDEERR;
	DWM1000_write32bitoffsetregister(SYS_STATUS_ID, 0, DWM1000local.sysstatus);

	DWM1000_write32bitoffsetregister(SYS_CFG_ID, 0x00, DWM1000local.sysCFGreg);			// set PHR-Mode in 0x04 register, set the hole 0x04 register
}

void DWM1000_receiver()
{
	DWM1000_new_receive();
	DWM1000_permanent_receive(true);
	DWM1000_start_Receive();
}

void DWM1000_start_Receive()
{
	DWM1000local.sysctrl |= SYS_CTRL_RXENAB;												// RX enable bit on // when frame check: set SYS_CTRL_SFCST
	DWM1000_write32bitoffsetregister(SYS_CTRL_ID, 0, DWM1000local.sysctrl);
}


float DWM1000_get_Receive_power()
{
	uint8_t cirPwr[2];
	uint8_t rxFrameInfo[4];

	unsigned long twoPower17 = 131072;
	unsigned int C, N;
	float A, corrFac;

	DWM1000_spi_read_burst_reg(RX_FQUAL_ID, 0x06 , cirPwr, 2);
	DWM1000_spi_read_burst_reg(RX_FINFO_ID, 0, rxFrameInfo, 4);

	C = (unsigned int)cirPwr[0] | ((unsigned int)cirPwr[1] << 8);
	N = (((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4);

	if(DWM1000local.prfIndex == 0)																			// 16 MHz PRF
	{
		A = 115.72;
		corrFac = 2.3334;
	}
	else
	{
		A       = 121.74;
		corrFac = 1.1667;
	}

	float estRxPwr = 10.0*log10(((float)C*(float)twoPower17)/((float)N*(float)N))-A;
	if(estRxPwr <= -88)
	{
		return estRxPwr;
	}
	else
	{
		estRxPwr += (estRxPwr+88)*corrFac;
	}

	return estRxPwr;
}


void DWM1000_correct_timestamp(uint64_t *timestamp)
{
	// base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
	float rxPowerBase     = -(DWM1000_get_Receive_power()+61.0f)*0.5f;
	int   rxPowerBaseLow  = (int)rxPowerBase;
	int   rxPowerBaseHigh = rxPowerBaseLow+1;
	if(rxPowerBaseLow < 0)
	{
		rxPowerBaseLow  = 0;
		rxPowerBaseHigh = 0;
	}
	else if(rxPowerBaseHigh > 17)
	{
		rxPowerBaseLow  = 17;
		rxPowerBaseHigh = 17;
	}
	// select range low/high values from corresponding table
	int rangeBiasHigh;
	int rangeBiasLow;

	if(DWM1000local.chan == 4 || DWM1000local.chan == 7)
	{
		// 900 MHz receiver bandwidth
		if(DWM1000local.prfIndex == 0)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow  = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		}
		else if(DWM1000local.prfIndex == 1)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow  = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else
		{
			// TODO proper error handling
		}
	}
	else
	{
		// 500 MHz receiver bandwidth
		if(DWM1000local.prfIndex == 0)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
			rangeBiasLow  = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
		}
		else if(DWM1000local.prfIndex == 1)
		{
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
			rangeBiasLow  = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
		}
		else
		{
			// TODO proper error handling
		}
	}
	// linear interpolation of bias values
	float rangeBias = rangeBiasLow+(rxPowerBase-rxPowerBaseLow)*(rangeBiasHigh-rangeBiasLow);
	// range bias [mm] to timestamp modification value conversion
	uint64_t adjustmentTime;
	adjustmentTime = (int)(rangeBias * DISTANCE_OF_RADIO_INV*0.001f);
	// apply correction

	*timestamp += adjustmentTime;
}

int64_t DWM1000_check_time_overflow(int64_t timeoverflow)
{
	if(timeoverflow < 0)
	{
		timeoverflow += TIME_OVERFLOW;
	}
	return timeoverflow;
}

/*
//for debuging, Yitong
void LED_blink_yitong(int ms, int num){
    for(int ij = 0; ij< num; ij++){
            LED_setLED(COL_RED);
            RTC_delay_ms(ms);
            LED_clearLED();
            RTC_delay_ms(ms);//+++++++++++++++++++++!!!!!!!!!!!!!!!!!!!!!Yitong
    }
}
*/

void DWM1000_UWB_NODE(uint16_t measurement_numbers)
{
	localization = STATE_UWB;
	uint32_t MY_NODE_ID = NODE_ID;
	uint8_t rx_buffer[FINAL_MESSAGE_LENGTH] = {0};
	uint32_t RANGE_TAG_ID = 0x00;;

	uint8_t MSG_TYPE = FUNC_CODE_POLL;
	uint64_t poll_tx_timestamp = 0;
	uint64_t resp_rx_timestamp = 0;
	uint16_t count1 = 45;  //3... 45 Patrick set 45, too much. Yitong  0x16470720 >>
	uint8_t count2 = 1;
	uint8_t count3 = 0;	// yitong
	uint8_t count4 = 0;	// yitong

	switch (NODE_ID)
	{
		case 0x2020:
			count3 = 2;	// yitong
			count4 = 2;	// yitong
		break;

		case 0x3E3E:
			count3 = 8;	// yitong
			count4 = 8;	// yitong
		break;

		case 0x4D4D:
			count3 = 16; //14;	// yitong
			count4 = 16; //14;	// yitong
		break;

		case 0x5A5A:
			count3 = 22; //20;	// yitong
			count4 = 22; //20;	// yitong
		break;

		case 0x6E6E:
			count3 = 28;	// yitong
			count4 = 28;	// yitong
		break;

		case 0x1C1C:
			count3 = 16;	// yitong REPLACE 4D4D WITH 1C1C
			count4 = 16;	// yitong
		break;

		default:
		break;
	}


	uint8_t debug_num = 205; // Yitong
	//RTC_delay_ms(100); // Yitong

	state1 = RECEIVER_ON_STATE;

	while(((measurement_numbers > 0 && count1 > 0) && count2 > 0) && (count3>0 && count4>0)) // Yitong 0x12380721 <<
	//while(((measurement_numbers > 0 && count1 > 0) && count2 > 0)) // Yitong 0x12380721 <<
	{
		switch (state1)
		{
			case RECEIVER_ON_STATE:
				//debug_num =  debug_num + 1;
				//debug_num =  debug_num + 10;
				DWM1000_receiver();
				//--blink_LED(2,3);  //yitong
				count4 = count4 - 1;  // Yitong, to prevent loooping forever between RECEIVER_ON_STATE to STATE_WAIT_BLINK_RECEIVE
				state1 = STATE_WAIT_BLINK_RECEIVE;
				RTC_start(600); //600, 800 and 1000 works, but out of no reasons RTC_TIMEOUT will nerver set to be true.// too long, change from 'RTC_start(6000);' by Yitong
			break;

			case STATE_WAIT_BLINK_RECEIVE:
				// TODO, SET LED BLINK HERE // <<<<<<<<<<<<<<<<<<<<<<<<  Yitong
				debug_num++;
				if(RTC_TIMEOUT)
				{
					// TODO, SET LED BLINK HERE // <<<<<<<<<<<<<<<<<<<<<<<<  Yitong
					count2 = 0;   // <<<<<<<<<<<<<<<<<<<<<<<<  Yitong, here dontt work 0x12380721 >>
					//debug_num = debug_num + 10; //Yitong, for debug
				}
			break;

			case STATE_BLINK_RECEIVE:
				//----blink_LED(1,2);  //yitong
				MSG_TYPE = FUNC_CODE_BLINK;
				state1 = DWM1000_blink_receive_state(rx_buffer, &RANGE_TAG_ID, &MY_NODE_ID, &MSG_TYPE);
			break;

			case STATE_POLL_TRANSMIT:
				//----blink_LED(3,2);  //yitong
				count3 = count3 - 1;  // Yitong, to prevent loooping forever between STATE_POLL_TRANSMIT to STATE_WAIT_RESPONSE_RECEIVE
				state1 = DWM1000_poll_transmit_state(RANGE_TAG_ID, MY_NODE_ID, &poll_tx_timestamp);
				RTC_start(80); //30...80
			break;

			case STATE_WAIT_RESPONSE_RECEIVE:
			   /*
				for(int ij = 0; ij< 100; ij++){
			            LED_setLED(COL_RED);
			            LED_clearLED();
			    }*/
				if(RTC_TIMEOUT)
				{
					count1 = count1 - 1;  // Yitong  0x16470720 <<
					state1 = STATE_POLL_TRANSMIT;
				}
			break;

			case STATE_RESPONSE_RECEIVE:
				//--blink_LED(5,2);  //yitong
				MSG_TYPE = FUNC_CODE_RESPONSE;
				state1 = DWM1000_response_receive_state(rx_buffer, &RANGE_TAG_ID, &MY_NODE_ID, &MSG_TYPE, &resp_rx_timestamp);
			break;

			case STATE_FINAL_TRANSMIT:
				//blink_LED(2,6);  //yitong
				DWM1000_send_final_message(FINAL_message, RANGE_TAG_ID, MY_NODE_ID, FINAL_MESSAGE_LENGTH, poll_tx_timestamp, resp_rx_timestamp);
				measurement_numbers--;
				state1 = STATE_POLL_TRANSMIT;
				RTC_delay_ms(10);
			break;

			default:
			break;
		}
	}
}

void DWM1000_UWB_TAG(float *range, uint16_t measurement_numbers, uint32_t *NODE_ADD)
{
	localization = STATE_UWB;
	uint8_t MSG_TYPE = 0;
	uint32_t MY_TAG_ID = TAG_ID;

	int i = 0;
	uint8_t rx_buffer[FINAL_MESSAGE_LENGTH]= {0};
	uint64_t poll_rx_timestamp = 0;
	uint64_t poll_tx_timestamp = 0;
	uint64_t resp_tx_timestamp = 0;
	uint64_t resp_rx_timestamp = 0;
	uint64_t final_rx_timestamp = 0;
	uint64_t final_tx_timestamp = 0;

	//DWM1000_Chip_INIT();

	while(measurement_numbers > 0)
	{
		switch (state)
		{
			case STATE_RECEIVER_ON:
				DWM1000_receiver();
				state = STATE_RECEIVE_MODE;
			break;

			case STATE_RECEIVE_MODE:
			break;

			case STATE_POLL_RECEIVE:
				MSG_TYPE = FUNC_CODE_POLL;
				state = DWM1000_poll_receive_state(rx_buffer, NODE_ADD, &MY_TAG_ID, &MSG_TYPE, &poll_rx_timestamp);
			break;

			case STATE_RESPONSE_TRANSMIT:
				state = DWM1000_response_transmit_state(*NODE_ADD, MY_TAG_ID, &resp_tx_timestamp);
			break;

			case STATE_WAIT_FINAL_RECEIVE:
			break;

			case STATE_FINAL_RECEIVE:
				memset(rx_buffer, 0, 27);
				DWM1000_get_receive_message(rx_buffer);

				MSG_TYPE = FUNC_CODE_FINAL;
				if (DWM1000_check_receive_message(rx_buffer, NODE_ADD, &MY_TAG_ID, &MSG_TYPE) == 0)
				{
					final_rx_timestamp = DWM1000_get_rx_timestamp_40bit();
					DWM1000_get_message_array_bytes(&rx_buffer[POLL_TX_TIMESTAMP_POSITION], &poll_tx_timestamp, 5);			// Get timestamps embedded in the final message
					DWM1000_get_message_array_bytes(&rx_buffer[RESP_RX_TIMESTAMP_POSITION], &resp_rx_timestamp, 5);
					DWM1000_get_message_array_bytes(&rx_buffer[FINAL_TX_TIMESTAMP_POSITION], &final_tx_timestamp, 5);

					range[i++] = DWM1000_compute_range_asymmetric(poll_tx_timestamp, poll_rx_timestamp, resp_tx_timestamp, resp_rx_timestamp, final_tx_timestamp, final_rx_timestamp);
					measurement_numbers--;
					state = STATE_RECEIVER_ON;
				}
				else
				{
					state = STATE_RECEIVER_ON;
				}
			break;

			default:
			break;
		}
	}
}


void DWM1000_Chip_INIT()
{
	while(DWM1000_init());																	// init DWM1000
	DWM1000_configuration2(&config);														// configuration of DWM1000
	DWM1000_set_GPIOs_sec_mode();
	DWM1000_config_sleep_mode(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);
}

float DWM1000_compute_range_asymmetric(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts, uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts)
{
	int64_t round1, round2, reply1, reply2, tof_dev_time_units;
	float tof, distance;

	char UART_data[256] = {0};

	round1 = (resp_rx_ts - poll_tx_ts);
	round2 = (final_rx_ts - resp_tx_ts);
	reply2 = (final_tx_ts - resp_rx_ts);
	reply1 = (resp_tx_ts - poll_rx_ts);

	round1 = DWM1000_check_time_overflow(round1);
	round2 = DWM1000_check_time_overflow(round2);
	reply2 = DWM1000_check_time_overflow(reply2);
	reply1 = DWM1000_check_time_overflow(reply1);

	tof_dev_time_units = (int64_t)((round1 * round2 - reply2 * reply1) / (round1 + round2 + reply2 + reply1));
	tof = tof_dev_time_units * DWT_TIME_UNITS;
	distance = (tof * SPEED_OF_LIGHT);

	sprintf(UART_data, "DIST: %ld cm", (long int) (distance * 100));
	UART_WriteString(UART_data, strlen(UART_data));

	return distance;
}

float DWM1000_compute_range_symmetric(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts, uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts)
{
	int64_t round1, round2, reply1, reply2, tof_dev_time_units;
	float tof, distance;

	char UART_data[256] = {0};

	round1 = (resp_rx_ts - poll_tx_ts);
	round2 = (final_rx_ts - resp_tx_ts);
	reply2 = (final_tx_ts - resp_rx_ts);
	reply1 = (resp_tx_ts - poll_rx_ts);

	round1 = DWM1000_check_time_overflow(round1);
	round2 = DWM1000_check_time_overflow(round2);
	reply2 = DWM1000_check_time_overflow(reply2);
	reply1 = DWM1000_check_time_overflow(reply1);

	tof_dev_time_units = (int64_t)((round1) - (reply2) + (round2) - (reply1)) * 0.25f;
	tof = tof_dev_time_units * DWT_TIME_UNITS;
	distance = (tof * SPEED_OF_LIGHT);

	sprintf(UART_data, "DIST: %ld cm", (long int) (distance * 100));
	UART_WriteString(UART_data, strlen(UART_data));

	return distance;
}


enum states_tag DWM1000_poll_receive_state(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE, uint64_t *poll_rx_ts)
{
	memset(rx_buffer, 0, 27);
	DWM1000_get_receive_message(rx_buffer);

	if (DWM1000_check_receive_message(rx_buffer, SOURCE_ID, DESTIN_ID, MSG_TYPE) == 0)					// if we have a good frame (1) or an error (0)
	{
		*poll_rx_ts = DWM1000_get_rx_timestamp_40bit();
		return STATE_RESPONSE_TRANSMIT;
	}
	else
	{
		return STATE_RECEIVER_ON;
	}
}

enum states_tag DWM1000_response_transmit_state(uint32_t DESTIN_ID, uint32_t SOURCE_ID, uint64_t *resp_tx_ts)
{
	if (DWM1000_send_response_message(RESP_message, DESTIN_ID, SOURCE_ID, RESPONSE_MESSAGE_LENGTH)==0)
	{
		*resp_tx_ts = DWM1000_get_tx_timestamp_40bit();
		return STATE_WAIT_FINAL_RECEIVE;
	}
	else
	{
		return STATE_RECEIVER_ON;
	}
}

enum states_tag DWM1000_poll_transmit_state(uint32_t DESTIN_ID, uint32_t SOURCE_ID, uint64_t *poll_tx_ts)
{
	DWM1000_receiver();
	DWM1000_send_poll_message(POLL_message, DESTIN_ID, SOURCE_ID, POLL_MESSAGE_LENGTH);			// send poll message to TAG immediate
	*poll_tx_ts = DWM1000_get_tx_timestamp_40bit();

	return  STATE_WAIT_RESPONSE_RECEIVE;
}

enum states_tag DWM1000_blink_receive_state(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE)
{
	memset(rx_buffer, 0, 27);
	DWM1000_get_receive_message(rx_buffer);														// if we have a good frame (1) or an error (0)

	if (DWM1000_check_receive_message(rx_buffer, SOURCE_ID, DESTIN_ID, MSG_TYPE) == 0)
	{
		return STATE_POLL_TRANSMIT;
	}
	else
	{
		return RECEIVER_ON_STATE;
	}
}

enum states_tag DWM1000_response_receive_state(uint8_t *rx_buffer, uint32_t *SOURCE_ID, uint32_t *DESTIN_ID, uint8_t *MSG_TYPE, uint64_t *resp_rx_ts)
{
	memset(rx_buffer, 0, 27);
	DWM1000_get_receive_message(rx_buffer);														// if we have a good frame (1) or an error (0)

	if (DWM1000_check_receive_message(rx_buffer, SOURCE_ID, DESTIN_ID, MSG_TYPE) == 0)
	{
		*resp_rx_ts = DWM1000_get_rx_timestamp_40bit();
		return STATE_FINAL_TRANSMIT;
	}
	else
	{
		return STATE_POLL_TRANSMIT;
	}
}
