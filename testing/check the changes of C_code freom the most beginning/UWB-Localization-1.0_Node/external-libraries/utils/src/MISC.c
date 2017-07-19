#include "MISC.h"

#include "UART.h"  // TODO: REMOVE!!
#include <stdio.h>  // TODO: REMOVE!!

static volatile uint32_t msTicks; /* counts 1ms timeTicks */

/*
 * LED Functions
 */

//
void LED_setLED(uint8_t color) {
	LED_clearLED();

	#if 	REVISION == IDENTIFIER_REV_1

		switch (color) {
		default:
		case COL_RED:
			GPIO_PinOutSet(LED_RED_PORT, LED_RED_PIN);
			break;
		case COL_GREEN:
			GPIO_PinOutSet(LED_GREEN_PORT, LED_GREEN_PIN);
			break;
		//case COL_BLUE:
			//GPIO_PinOutSet(LED_BLUE_PORT, LED_BLUE_PIN);
			//break;
		}

	#elif	REVISION == IDENTIFIER_REV_2 || REVISION == IDENTIFIER_REV_3 || REVISION == TABLE_REV_1
		switch (color) {
			default:
			case COL_RED:
				GPIO_PinOutSet(LED_RED_PORT, LED_RED_PIN);
				break;
			case COL_GREEN:
				GPIO_PinOutSet(LED_GREEN_PORT, LED_GREEN_PIN);
				break;
		}

	#elif 	REVISION == BASE_REV_1

		switch (color) {
		default:
		case COL_RED:
			GPIO_PinOutSet(LED_RED_PORT, LED_RED_PIN);
			break;
		case COL_GREEN:
			GPIO_PinOutSet(LED_GREEN_PORT, LED_GREEN_PIN);
			break;
		case COL_BLUE:
			GPIO_PinOutSet(LED_BLUE_PORT, LED_BLUE_PIN);
			break;
		}

#elif 	REVISION == BASE_REV_2

		switch (color) {
		default:
		case COL_RED:
			GPIO_PinOutSet(LED_RED_PORT, LED_RED_PIN);
			break;
		case COL_GREEN:
			GPIO_PinOutSet(LED_GREEN_PORT, LED_GREEN_PIN);
			break;
		case COL_BLUE:
			break;
		}


	#endif

}

//
void LED_clearLED(void) {
	#if 	REVISION == IDENTIFIER_REV_1
		GPIO_PinOutClear(LED_RED_PORT, LED_RED_PIN);
		GPIO_PinOutClear(LED_GREEN_PORT, LED_GREEN_PIN);
		//GPIO_PinOutClear(LED_BLUE_PORT, LED_BLUE_PIN);

	#elif	REVISION == IDENTIFIER_REV_2 || REVISION == IDENTIFIER_REV_3
		GPIO_PinOutClear(LED_RED_PORT, LED_RED_PIN);
		GPIO_PinOutClear(LED_GREEN_PORT, LED_GREEN_PIN);

	#elif 	REVISION == TABLE_REV_1
		GPIO_PinOutClear(LED_RED_PORT, LED_RED_PIN);
		GPIO_PinOutClear(LED_GREEN_PORT, LED_GREEN_PIN);

	#elif 	REVISION == BASE_REV_1
		GPIO_PinOutClear(LED_RED_PORT, LED_RED_PIN);
		GPIO_PinOutClear(LED_GREEN_PORT, LED_GREEN_PIN);
		GPIO_PinOutClear(LED_BLUE_PORT, LED_BLUE_PIN);

	#elif 	REVISION == BASE_REV_2
		GPIO_PinOutClear(LED_RED_PORT, LED_RED_PIN);
		GPIO_PinOutClear(LED_GREEN_PORT, LED_GREEN_PIN);
	#endif
}

/*
 * Delay Functions
 */
//static volatile uint32_t msTicks; /* counts 1ms timeTicks */
//
void SysTick_Handler(void) {
	msTicks++; /* increment counter necessary in Delay()*/
	millis++;
}

//
void SysTick_Delay_init(void) {
	/* Setup SysTick Timer for 1 msec interrupts */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
		while (1)
			;
	}
}

//
void SysTick_Delay(uint32_t dlyTicks) {
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks)
		;
}

/*
 * ADC Functions
*/


uint16_t ADC_get_bat_voltage(void) {
//
//	#if REVISION == TABLE_REV_1
//	GPIO_PinOutSet(LED_VDEN_PORT, LED_VDEN_PIN);
//	#endif
//
//	/* Start ADC */
//	ADC_Start(ADC0, adcStartSingle);
//
//	/* Wait while conversion is active */
//	while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;
//
//	/* Get ADC result */
//	uint32_t sample = ADC_DataSingleGet(ADC0);
//
//	#if REVISION == TABLE_REV_1
//	GPIO_PinOutClear(LED_VDEN_PORT, LED_VDEN_PIN);
//	#endif
//
//	/* Calculate supply voltage relative to 1.25V reference and return it */
//	return (uint16_t) ((sample * 1250 * 3) / 4096);
}
