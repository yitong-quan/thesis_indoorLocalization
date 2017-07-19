#include "TIMER.h"

void TIMER_init(TIMER_TypeDef *timer, CMU_Clock_TypeDef timer_clock,
		IRQn_Type timer_irq) {
	TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;

	/* Enable clock for the given timer module. */
	CMU_ClockEnable(timer_clock, true);

	timer_init.enable = false;
	timer_init.debugRun = true;
	timer_init.prescale = timerPrescale1024;
	timer_init.clkSel = timerClkSelHFPerClk;
	timer_init.fallAction = timerInputActionNone;
	timer_init.riseAction = timerInputActionNone;
	timer_init.mode = timerModeUp;
	timer_init.oneShot = true;

	TIMER_IntEnable(timer, TIMER_IF_OF);
	NVIC_EnableIRQ(timer_irq);

	/* Configure the given timer. */
	TIMER_Init(timer, &timer_init);

	TIMER_stop(timer);
}

void TIMER_init_us(TIMER_TypeDef *timer, CMU_Clock_TypeDef timer_clock,
		IRQn_Type timer_irq) {
	TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;

	/* Enable clock for the given timer module. */
	CMU_ClockEnable(timer_clock, true);

	timer_init.enable = false;
	timer_init.debugRun = true;
	timer_init.prescale = timerPrescale1;
	timer_init.clkSel = timerClkSelHFPerClk;
	timer_init.fallAction = timerInputActionNone;
	timer_init.riseAction = timerInputActionNone;
	timer_init.mode = timerModeUp;
	timer_init.oneShot = true;

	TIMER_IntEnable(timer, TIMER_IF_OF);
	NVIC_EnableIRQ(timer_irq);

	/* Configure the given timer. */
	TIMER_Init(timer, &timer_init);

	TIMER_stop_us(timer);
}

void TIMER_start(TIMER_TypeDef *timer, uint16_t timeout_ms) {
	uint32_t timeout;

	/* TODO: Magic number: 3195 = max timeout value. @21MHz*/
	if (timeout_ms > 3195)
		timeout_ms = 3195;

	/* TODO: Magic number: 20508 = 1 second in counter value. @21MHz*/
	timeout = (20508 * timeout_ms) / 1000UL;

	TIMER_TIMEOUT = false;

	if (timeout) {
		TIMER_CounterSet(timer, 0);
		TIMER_TopSet(timer, timeout);
		TIMER_Enable(timer, true);
	}
}

void TIMER_start_us(TIMER_TypeDef *timer, uint16_t timeout_us) {
	uint32_t timeout;

	/* Magic number: 3120 = max timeout value. @21MHz*/
	if (timeout_us > 3120)
		timeout_us = 3120;

	/* Minimum time is 1 us */
	if (timeout_us < 1) {
		timeout_us = 1;
	}

	/* Magic number: 21 = 1 microsecond in counter value. @21MHz*/
	timeout = (21 * timeout_us);

	TIMER_US_TIMEOUT = false;

	if (timeout) {
		TIMER_CounterSet(timer, 0);
		TIMER_TopSet(timer, timeout);
		TIMER_Enable(timer, true);
	}
}

void TIMER_stop(TIMER_TypeDef *timer) {
	TIMER_Enable(timer, false);
	TIMER_CounterSet(timer, 0);
	TIMER_TIMEOUT = false;
}

void TIMER_stop_us(TIMER_TypeDef *timer) {
	TIMER_Enable(timer, false);
	TIMER_CounterSet(timer, 0);
	TIMER_US_TIMEOUT = false;
}

void TIMER_delay_ms(TIMER_TypeDef *timer, uint16_t timeout_ms, bool use_EM1_mode) {
	TIMER_start(timer, timeout_ms);
	while (!TIMER_TIMEOUT) {
		if(use_EM1_mode) {
			__disable_irq();
			EMU_EnterEM1();  // Sleep mode
			__enable_irq();
		}
	}
	TIMER_stop(timer );
}

void TIMER_delay_us(TIMER_TypeDef *timer, uint16_t timeout_us) {
	TIMER_start_us(timer, timeout_us);
	while (!TIMER_US_TIMEOUT);
	TIMER_stop_us(timer );
}

//
void TIMER0_IRQHandler(void) {
	__disable_irq();
	TIMER_TIMEOUT = true;
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	__enable_irq();
}

void TIMER2_IRQHandler(void) {
	__disable_irq();
	TIMER_US_TIMEOUT = true;
	TIMER_IntClear(TIMER2, TIMER_IF_OF);
	__enable_irq();
}
