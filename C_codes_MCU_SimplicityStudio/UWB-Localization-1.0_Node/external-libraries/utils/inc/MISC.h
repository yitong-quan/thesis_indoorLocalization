#ifndef UTILS_MISC_H_
#define UTILS_MISC_H_

#include "config.h"
#include "revisionDef.h"
#include <stdio.h>  // TODO: REMOVE!!

#include "em_gpio.h"
#include "em_cmu.h"
#include "em_adc.h"

// LED DEFINES

	#define LED_GREEN_PORT 	gpioPortD
	#define LED_GREEN_PIN 	5
	#define LED_RED_PORT 	gpioPortA
	#define LED_RED_PIN 	1
	//#define LED_BLUE_PORT 	gpioPortB
	//#define LED_BLUE_PIN 	11

#if	REVISION == IDENTIFIER_REV_1

	#define LED_GREEN_PORT 	gpioPortB
	#define LED_GREEN_PIN 	14
	#define LED_RED_PORT 	gpioPortB
	#define LED_RED_PIN 	13
	//#define LED_BLUE_PORT 	gpioPortB
	//#define LED_BLUE_PIN 	11

#elif	(REVISION == IDENTIFIER_REV_2) || (REVISION == IDENTIFIER_REV_3)

	#define LED_RED_PORT 	gpioPortB
	#define LED_RED_PIN 	11
	#define LED_GREEN_PORT 	gpioPortB
	#define LED_GREEN_PIN 	13

#elif 	REVISION == TABLE_REV_1

	#define LED_RED_PORT 	gpioPortA
	#define LED_RED_PIN 	1
	#define LED_GREEN_PORT 	gpioPortD
	#define LED_GREEN_PIN 	5
	//#define LED_VDEN_PORT 	gpioPortA
	//#define LED_VDEN_PIN 	1


#elif	REVISION == BASE_REV_1

	#define LED_RED_PORT 	gpioPortB
	#define LED_RED_PIN 	14
	#define LED_GREEN_PORT 	gpioPortB
	#define LED_GREEN_PIN 	13
	#define LED_BLUE_PORT 	gpioPortB
	#define LED_BLUE_PIN 	11

#elif	REVISION == BASE_REV_2

	#define LED_RED_PORT 	gpioPortB
	#define LED_RED_PIN 	14
	#define LED_GREEN_PORT 	gpioPortB
	#define LED_GREEN_PIN 	13

#endif

#define COL_RED 	0
#define COL_GREEN 	1
#define COL_BLUE 	2
#define COL_YELLOW 	3

volatile uint32_t millis; /* counts 1ms timeTicks */

/*
 *
 */
void LED_setLED(uint8_t color);

/*
 *
 */
void LED_clearLED(void);

/*
 *
 */
void SysTick_Handler(void);

/*
 *
 */
void SysTick_Delay_init(void);

/*
 *
 */
void SysTick_Delay(uint32_t dlyTicks);

/*
 *
 */
uint16_t ADC_get_bat_voltage(void);

#endif /* UTILS_MISC_H_ */
