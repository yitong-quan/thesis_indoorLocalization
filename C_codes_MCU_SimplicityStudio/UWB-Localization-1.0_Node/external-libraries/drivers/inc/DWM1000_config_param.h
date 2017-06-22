/*
 * DWM1000_config_param.h
 *
 *  Created on: 13.05.2016
 *      Author: Guillermo
 */

#ifndef DWM1000_CONFIG_PARAM_H_
#define DWM1000_CONFIG_PARAM_H_

/* LDE Replica Coefficient configuration (Register 0x2E:2804 Leading Edge Detection Interface)
 * LDE_REPC configuration needs to be set depending on the receiver preamble code configuration
 * These values apply to 850 kbps and 6.8 Mbps data rates only, for 110 kbps we have to divide the value by 8
 */

#define LDE_REBLICA_COEFF_1		0x5998
#define LDE_REBLICA_COEFF_2		0x5998
#define LDE_REBLICA_COEFF_3		0x51EA
#define LDE_REBLICA_COEFF_4		0x428E
#define LDE_REBLICA_COEFF_5		0x451E
#define LDE_REBLICA_COEFF_6		0x2E14
#define LDE_REBLICA_COEFF_7		0x8000
#define LDE_REBLICA_COEFF_8		0x51EA
#define LDE_REBLICA_COEFF_9		0x28F4
#define LDE_REBLICA_COEFF_10	0x3332
#define LDE_REBLICA_COEFF_11	0x3AE0
#define LDE_REBLICA_COEFF_12	0x3D70
#define LDE_REBLICA_COEFF_13	0x3AE0
#define LDE_REBLICA_COEFF_14	0x35C2
#define LDE_REBLICA_COEFF_15	0x2B84
#define LDE_REBLICA_COEFF_16	0x35C2
#define LDE_REBLICA_COEFF_17	0x3332
#define LDE_REBLICA_COEFF_18	0x35C2
#define LDE_REBLICA_COEFF_19	0x35C2
#define LDE_REBLICA_COEFF_20	0x47AE
#define LDE_REBLICA_COEFF_21	0x3AE0
#define LDE_REBLICA_COEFF_22	0x3850
#define LDE_REBLICA_COEFF_23	0x30A2
#define LDE_REBLICA_COEFF_24	0x3850

/* LDE algorithm parameters:
 * LDE_CFG1: Number of Standard Deviations (NSTDEV), Peak Multiplier (PMULT)
 * LDE_CFG2: 16 MHz PRF: 0x1607 or for 64 MHz PRF: 0x0607
 */

#define PEAK_MULTPLIER 			(0x60)								//3 -> (0x3 * 32) & 0x00E0
#define N_STD_FACTOR 			(13)
#define LDE_PARAM_CFG1			(PEAK_MULTPLIER | N_STD_FACTOR)

#define LDE_PARAM_CFG2_16 		0x1607
#define LDE_PARAM_CFG2_64 		0x0607


/* Frequency synthesiser PLL configuration:
 * FS_PLLCFG: values dependig on the used channel
 */
#define PLL_CONFIG_CHAN_1		0x09000407UL
#define PLL_CONFIG_CHAN_2_4		0x08400508UL
#define PLL_CONFIG_CHAN_3		0x08401009UL
#define PLL_CONFIG_CHAN_5_7		0x0800041DUL

/* Frequency synthesiser PLL tune:
 * FS_PLLCFG: values dependig on the used channel
 */
#define PLL_TUNE_CHAN_1			0x1E
#define PLL_TUNE_CHAN_2_4		0x26
#define PLL_TUNE_CHAN_3			0x56
#define PLL_TUNE_CHAN_5_7		0xBE

/* Analog RX Control Register
 * Choose RX receiver config depending on used Channel
 * Wide band or Narrow band
 */

#define RX_CTRL_CHAN_1_2_3_5	0xD8
#define RX_CTRL_CHAN_4_7		0xBC

/* Digital Tuning Register 1a
 * This value has to be changed depending on RXPRF configuration
 */
#define DRX_TUNE1a_16MHZ_PRF 	0x0087
#define DRX_TUNE1a_64MHZ_PRF 	0x008D

// DRX TUNE2

#define DRX_TUNE2_16MHZPRF_PAC8		0x311A002DUL
#define DRX_TUNE2_16MHZPRF_PAC16	0x331A0052UL
#define DRX_TUNE2_16MHZPRF_PAC32	0x351A009AUL
#define DRX_TUNE2_16MHZPRF_PAC64	0x371A011DUL

#define DRX_TUNE2_64MHZPRF_PAC8		0x313B006BUL
#define DRX_TUNE2_64MHZPRF_PAC16	0x333B00BEUL
#define DRX_TUNE2_64MHZPRF_PAC32	0x353B015EUL
#define DRX_TUNE2_64MHZPRF_PAC64	0x373B0296UL

// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
const uint8_t BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112};
const uint8_t BIAS_500_64[] = {110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86};
const uint8_t BIAS_900_16[] = {137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
const uint8_t BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};

static const uint8_t BIAS_500_16_ZERO = 10;
static const uint8_t BIAS_500_64_ZERO = 8;
static const uint8_t BIAS_900_16_ZERO = 7;
static const uint8_t BIAS_900_64_ZERO = 7;

static const float DISTANCE_OF_RADIO     = 0.0046917639786159f;
static const float DISTANCE_OF_RADIO_INV = 213.139451293f;

static const float TIME_RES     = 0.000015650040064103f;
static const float TIME_RES_INV = 63897.6f;

// timer/counter overflow (40 bits) -> 4overflow approx. every 17.2 seconds
static const int64_t TIME_OVERFLOW = 0x10000000000;							//1099511627776LL
static const int64_t TIME_MAX = 0xffffffffff;

#endif /* DWM1000_CONFIG_PARAM_H_ */
