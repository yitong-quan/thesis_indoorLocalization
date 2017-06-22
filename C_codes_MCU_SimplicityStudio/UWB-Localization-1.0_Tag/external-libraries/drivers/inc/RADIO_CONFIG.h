#ifndef UTILS_RADIO_CONFIG_H
#define UTILS_RADIO_CONFIG_H

#include "CC1101.h"
#include "config.h"

/* Datarate 38.4 kBaud, Dev: 20kHz, Mod: GFSK, RX BW: 100 kHz, Optimized for sensitivity */
static const struct cc1101_config cc1101_config_38kbaud =
{

/* GDO2 output pin configuration. */
	.iocfg2 = 0x00,

	/* GDO1 output pin configuration. */
	.iocfg1 = 0x2e,

	/* GDO0 output pin configuration. */
	.iocfg0 = 0x06,

	/* RX FIFO and TX FIFO thresholds. */
	.fifothr = 0x0e,

	/* Sync word (high and low byte). */
	.sync1 = 0xd3,
	.sync0 = 0x91,

	/* Packet length. */
	.pktlen = 0xff,

	/* Packet automation control. */
	.pktctrl1 = 0x04,
	.pktctrl0 = 0x45,

	/* Device address. */
	.addr = 0x00,

	/* Channel number. */
	.channr = 0x00,

	/* Frequency synthesizer control. */
	.fsctrl1 = 0x06,
	.fsctrl0 = 0x00,

	/* Frequency control word (high, middle and low byte). */
	#if HF_FREQ == 915
		.freq2 = 0x23,
		.freq1 = 0x31,
		.freq0 = 0x3B,
	#elif HF_FREQ == 869
		.freq2 = 0x21,
		.freq1 = 0x71,
		.freq0 = 0x7A,
	#endif

	/* Modem configuration. */
	.mdmcfg4 = 0x4A,
	.mdmcfg3 = 0x83,
	.mdmcfg2 = 0x13,
	.mdmcfg1 = 0x22,
	.mdmcfg0 = 0xF8,

	/* Modem deviation setting. */
	.deviatn = 0x35,

	/* Main Radio Control State Machine configuration. */
	.mcsm2 = 0x07,
	#if USE_CSMA_CA
		.mcsm1 = 0x30,
	#else
		.mcsm1 = 0x30,
	#endif
	.mcsm0 = 0x18,

	/* Frequency Offset Compensation configuration. */
	.foccfg = 0x16,

	/* Bit Synchronization configuration. */
	.bscfg = 0x6C,

	/* AGC control. */
	.agcctrl2 = 0x43,
	.agcctrl1 = 0x40,
	.agcctrl0 = 0x91,

	/* Event 0 timeout (high and low byte). */
	.worevt1 = 0x87,
	.worevt0 = 0x6b,

	/* Wake On Radio control. */
	.worctrl = 0xFB,

	/* Front end RX configuration. */
	.frend1 = 0x56,

	/* Front end TX configuration. */
	.frend0 = 0x10,

	/* Frequency synthesizer calibration. */
	.fscal3 = 0xE9,
	.fscal2 = 0x2a,
	.fscal1 = 0x00,
	.fscal0 = 0x1f,

	/* RC oscillator configuration. */
	.rcctrl1 = 0x41,
	.rcctrl0 = 0x00,

	/* Frequency synthesizer calibration control. */
	.fstest = 0x59,

	/* Production test. */
	.ptest = 0x7f,

	/* AGC test. */
	.agctest = 0x3f,

	/* Various test settings. */
	.test2 = 0x88,
	.test1 = 0x31,
	.test0 = 0x09
};

/* Data rate 500 kBaud, Mod.: MSK, RX BW: 812 kHz, optimized for sensitivity. */
static const struct cc1101_config cc1101_config =
{
	/* GDO2 output pin configuration. */
	.iocfg2 = 0x00,

	/* GDO1 output pin configuration. */
	.iocfg1 = 0x2e,

	/* GDO0 output pin configuration. */
	.iocfg0 = 0x06,

	/* RX FIFO and TX FIFO thresholds. */
	.fifothr = 0x0e,

	/* Sync word (high and low byte). */
	.sync1 = 0xd3,
	.sync0 = 0x91,

	/* Packet length. */
	.pktlen = 0xff,

	/* Packet automation control. */
	.pktctrl1 = 0x04,
	.pktctrl0 = 0x45,

	/* Device address. */
	.addr = 0x00,

	/* Channel number. */
	.channr = 0x00,

	/* Frequency synthesizer control. */
	.fsctrl1 = 0x0e,
	.fsctrl0 = 0x00,

	/* Frequency control word (high, middle and low byte). */
	#if HF_FREQ == 915
		.freq2 = 0x23,
		.freq1 = 0x31,
		.freq0 = 0x3B,
	#elif HF_FREQ == 869
		.freq2 = 0x21,
		.freq1 = 0x71,
		.freq0 = 0x7A,
	#endif

	/* Modem configuration. */
	.mdmcfg4 = 0x0e,
	.mdmcfg3 = 0x3b,
	.mdmcfg2 = 0x73,
	.mdmcfg1 = 0x42,
	.mdmcfg0 = 0xf8,

	/* Modem deviation setting. */
	.deviatn = 0x00,

	/* Main Radio Control State Machine configuration. */
	.mcsm2 = 0x07,
	.mcsm1 = 0x00,
	.mcsm0 = 0x18,

	/* Frequency Offset Compensation configuration. */
	.foccfg = 0x1D,

	/* Bit Synchronization configuration. */
	.bscfg = 0x1c,

	/* AGC control. */
	.agcctrl2 = 0xc7,
	.agcctrl1 = 0x00,
	.agcctrl0 = 0xb0,

	/* Event 0 timeout (high and low byte). */
	.worevt1 = 0x87,
	.worevt0 = 0x6b,

	/* Wake On Radio control. */
	.worctrl = 0xf8,

	/* Front end RX configuration. */
	.frend1 = 0xb6,

	/* Front end TX configuration. */
	.frend0 = 0x10,

	/* Frequency synthesizer calibration. */
	.fscal3 = 0xea,
	.fscal2 = 0x2a,
	.fscal1 = 0x00,
	.fscal0 = 0x1f,

	/* RC oscillator configuration. */
	.rcctrl1 = 0x41,
	.rcctrl0 = 0x00,

	/* Frequency synthesizer calibration control. */
	.fstest = 0x59,

	/* Production test. */
	.ptest = 0x7f,

	/* AGC test. */
	.agctest = 0x3f,

	/* Various test settings. */
	.test2 = 0x88,
	.test1 = 0x31,
	.test0 = 0x09
};

static const struct cc1101_config cc1101_wakeup_config =  {
	/* GDO2 output pin configuration. */
	.iocfg2 = 0x02,

	/* GDO1 output pin configuration. */
	.iocfg1 = 0x2e,

	/* GDO0 output pin configuration. */
	.iocfg0 = 0x06,

	/* RX FIFO and TX FIFO thresholds. */
	.fifothr = 0x0e,

	/* Sync word (high and low byte). */
	.sync1 = 0xd3,
	.sync0 = 0x91,

	/* Packet length. */
	.pktlen = 0x02,

	/* Packet automation control. */
	.pktctrl1 = 0x00,
	.pktctrl0 = 0x01,

	/* Device address. */
	.addr = 0x00,

	/* Channel number. */
	.channr = 0x00,

	/* Frequency synthesizer control. */
	.fsctrl1 = 0x0c,
	.fsctrl0 = 0x00,

	/* Frequency control word (high, middle and low byte). */
	#if HF_FREQ == 915
		.freq2 = 0x23,
		.freq1 = 0x31,
		.freq0 = 0x3B,
	#elif HF_FREQ == 869
		.freq2 = 0x21,
		.freq1 = 0x71,
		.freq0 = 0x7A,
	#endif

	/* Modem configuration. */
	.mdmcfg4 = 0x8d,
	.mdmcfg3 = 0x3b,
	.mdmcfg2 = 0x32,
	.mdmcfg1 = 0x02,
	.mdmcfg0 = 0xf8,

	/* Modem deviation setting. */
	.deviatn = 0x15,

	/* Main Radio Control State Machine configuration. */
	.mcsm2 = 0x07,
	.mcsm1 = 0x30,
	.mcsm0 = 0x18,

	/* Frequency Offset Compensation configuration. */
	.foccfg = 0x1c,

	/* Bit Synchronization configuration. */
	.bscfg = 0x1c,

	/* AGC control. */
	.agcctrl2 = 0xc7,
	.agcctrl1 = 0x00,
	.agcctrl0 = 0xb0,

	/* Event 0 timeout (high and low byte). */
	.worevt1 = 0x87,
	.worevt0 = 0x6b,

	/* Wake On Radio control. */
	.worctrl = 0xf8,

	/* Front end RX configuration. */
	.frend1 = 0xb6,

	/* Front end TX configuration. */
	.frend0 = 0x17,

	/* Frequency synthesizer calibration. */
	.fscal3 = 0xea,
	.fscal2 = 0x2a,
	.fscal1 = 0x00,
	.fscal0 = 0x1f,

	/* RC oscillator configuration. */
	.rcctrl1 = 0x41,
	.rcctrl0 = 0x00,

	/* Frequency synthesizer calibration control. */
	.fstest = 0x59,

	/* Production test. */
	.ptest = 0x7f,

	/* AGC test. */
	.agctest = 0x3f,

	/* Various test settings. */
	.test2 = 0x81,
	.test1 = 0x35,
	.test0 = 0x09
};

static const struct cc1101_config cc1101_wakeup_config_narrow_band = {

		/* GDO2 output pin configuration. */
		.iocfg2 = 0x02,

		/* GDO1 output pin configuration. */
		.iocfg1 = 0x2e,

		/* GDO0 output pin configuration. */
		.iocfg0 = 0x06,

		/* RX FIFO and TX FIFO thresholds. */
		.fifothr = 0x0e,

		/* Sync word (high and low byte). */
		.sync1 = 0xd3,
		.sync0 = 0x91,

		/* Packet length. */
		.pktlen = 0x02,

		/* Packet automation control. */
		.pktctrl1 = 0x00,
		.pktctrl0 = 0x01,

		/* Device address. */
		.addr = 0x00,

		/* Channel number. */
		.channr = 0x00,

		/* Frequency synthesizer control. */
		.fsctrl1 = 0x0c,
		.fsctrl0 = 0x00,

		/* Frequency control word (high, middle and low byte). */
		#if HF_FREQ == 915
			.freq2 = 0x23,
			.freq1 = 0x31,
			.freq0 = 0x3B,
		#elif HF_FREQ == 869
			.freq2 = 0x21,
			.freq1 = 0x71,
			.freq0 = 0x7A,
		#endif

		/* Modem configuration. */
		// .mdmcfg4 = 0x2d => 250 kBaud/s
		// .mdmcfg4 = 0x2c => 125 kBaud/s
		// .mdmcfg4 = 0x2a =>  50 kBaud/s
		// .mdmcfg4 = 0x2a =>  38.4 kBaud/s
		// .mdmcfg3 = 0x3b => 250 kBaud/s
		// .mdmcfg3 = 0x3b => 125 kBaud/s
		// .mdmcfg3 = 0xf8 =>  50 kBaud/s
		// .mdmcfg3 = 0x83 =>  38.4 kBaud/s

		#if 	LF_FREQ == 125000
			.mdmcfg4 = 0x2d,
			.mdmcfg3 = 0x3b,
		#elif	LF_FREQ == 19200
			.mdmcfg4 = 0x2a,
			.mdmcfg3 = 0x83,
		#endif

		.mdmcfg2 = 0x30,
		.mdmcfg1 = 0x22,
		.mdmcfg0 = 0xf8,

		/* Modem deviation setting. */
		.deviatn = 0x62,

		/* Main Radio Control State Machine configuration. */
		.mcsm2 = 0x07,
		.mcsm1 = 0x30,
		.mcsm0 = 0x18,

		/* Frequency Offset Compensation configuration. */
		.foccfg = 0x1d,

		/* Bit Synchronization configuration. */
		.bscfg = 0x1c,

		/* AGC control. */
		.agcctrl2 = 0xc7,
		.agcctrl1 = 0x00,
		.agcctrl0 = 0xb0,

		/* Event 0 timeout (high and low byte). */
		.worevt1 = 0x87,
		.worevt0 = 0x6b,

		/* Wake On Radio control. */
		.worctrl = 0xfb,

		/* Front end RX configuration. */
		.frend1 = 0xb6,

		/* Front end TX configuration. */
		.frend0 = 0x17,

		/* Frequency synthesizer calibration. */
		//.fscal3 = 0xea => 250 kBaud/s
		//.fscal3 = 0xea => 125 kBaud/s
		//.fscal3 = 0xe9 => 50 kBaud/s
		//.fscal3 = 0xe9 => 38.4 kBaud/s
		#if 	LF_FREQ == 125000
			.fscal3 = 0xea,
		#elif	LF_FREQ == 19200
			.fscal3 = 0xe9,
		#endif

		.fscal2 = 0x2a,
		.fscal1 = 0x00,
		.fscal0 = 0x1f,

		/* RC oscillator configuration. */
		.rcctrl1 = 0x41,
		.rcctrl0 = 0x00,

		/* Frequency synthesizer calibration control. */
		.fstest = 0x59,

		/* Production test. */
		.ptest = 0x7f,

		/* AGC test. */
		.agctest = 0x3f,

		/* Various test settings. */
		.test2 = 0x81,
		.test1 = 0x35,
		.test0 = 0x09

};

static const struct cc1101_config cc1101_spectral_analysis = {

		/* GDO2 output pin configuration. */
		.iocfg2 = 0x02,

		/* GDO1 output pin configuration. */
		.iocfg1 = 0x2e,

		/* GDO0 output pin configuration. */
		.iocfg0 = 0x06,

		/* RX FIFO and TX FIFO thresholds. */
		.fifothr = 0x0e,

		/* Sync word (high and low byte). */
		.sync1 = 0xd3,
		.sync0 = 0x91,

		/* Packet length. */
		.pktlen = 0x02,

		/* Packet automation control. */
		.pktctrl1 = 0x00,
		.pktctrl0 = 0x01,

		/* Device address. */
		.addr = 0x00,

		/* Channel number. */
		.channr = 0x00,

		/* Frequency synthesizer control. */
		.fsctrl1 = 0x0c,
		.fsctrl0 = 0x00,

		/* Frequency control word (high, middle and low byte). */
		#if HF_FREQ == 915
			.freq2 = 0x23,
			.freq1 = 0x31,
			.freq0 = 0x3B,
		#elif HF_FREQ == 869
			.freq2 = 0x21,
			.freq1 = 0x71,
			.freq0 = 0x7A,
		#endif

		/* Modem configuration. */
		// .mdmcfg4 = 0x2d => 250 kBaud/s
		// .mdmcfg4 = 0x2c => 125 kBaud/s
		.mdmcfg4 = 0x2d,
		.mdmcfg3 = 0x3b,
		.mdmcfg2 = 0x30,
		.mdmcfg1 = 0x22,
		.mdmcfg0 = 0xf8,

		/* Modem deviation setting. */
		.deviatn = 0x62,

		/* Main Radio Control State Machine configuration. */
		.mcsm2 = 0x07,
		.mcsm1 = 0x30,
		.mcsm0 = 0x18,

		/* Frequency Offset Compensation configuration. */
		.foccfg = 0x1d,

		/* Bit Synchronization configuration. */
		.bscfg = 0x1c,

		/* AGC control. */
		.agcctrl2 = 0xc7,
		.agcctrl1 = 0x00,
		.agcctrl0 = 0xb0,

		/* Event 0 timeout (high and low byte). */
		.worevt1 = 0x87,
		.worevt0 = 0x6b,

		/* Wake On Radio control. */
		.worctrl = 0xfb,

		/* Front end RX configuration. */
		.frend1 = 0xb6,

		/* Front end TX configuration. */
		.frend0 = 0x17,

		/* Frequency synthesizer calibration. */
		.fscal3 = 0xea,
		.fscal2 = 0x2a,
		.fscal1 = 0x00,
		.fscal0 = 0x1f,

		/* RC oscillator configuration. */
		.rcctrl1 = 0x41,
		.rcctrl0 = 0x00,

		/* Frequency synthesizer calibration control. */
		.fstest = 0x59,

		/* Production test. */
		.ptest = 0x7f,

		/* AGC test. */
		.agctest = 0x3f,

		/* Various test settings. */
		.test2 = 0x81,
		.test1 = 0x35,
		.test0 = 0x09

};

#endif /* UTILS_RADIO_CONFIG_H */
