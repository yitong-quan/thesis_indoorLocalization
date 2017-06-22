#include "RTC.h"

//
void RTC_init(uint16_t timeout_ms) {
	RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

	rtcInit.enable = true; /* Enable RTC after init has run */
	rtcInit.comp0Top = true; /* Clear counter on compare match */
	rtcInit.debugRun = false; /* Counter shall keep running during debug halt. */

	/* Setting the compare value of the RTC */
	RTC_CompareSet(0, (((32768 * timeout_ms) / 1000) - 1));

	/* Enabling Interrupt from RTC */
	RTC_IntEnable(RTC_IFC_COMP0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Initialize the RTC */
	RTC_Init(&rtcInit);

	// Don't start after init
	RTC_Enable(false);
}

//
void RTC_start(uint16_t timeout_ms) {
	RTC_Enable(true);  // Start RTC (for timeout)
	RTC_CompareSet(0, (((32768 * timeout_ms) / 1000) - 1));
	RTC_CounterReset();
	RTC_TIMEOUT = false;
}

//
void RTC_reset() {
	RTC_CounterReset();
	RTC_TIMEOUT = false;
}

//
void RTC_delay_ms(uint16_t delay_ms) {
	RTC_start(delay_ms);
	while (!RTC_TIMEOUT) {
		__disable_irq();
		EMU_EnterEM2(true);  // Sleep mode
		__enable_irq();
	}
}

// RTC Compare Interrupt
void RTC_IRQHandler(void) {
	// Set Timeout Variable
	RTC_TIMEOUT = true;
	// Disable RTC
	RTC_Enable(false);
	/* Clear interrupt source */
	RTC_IntClear(RTC_IFC_COMP0);
}
