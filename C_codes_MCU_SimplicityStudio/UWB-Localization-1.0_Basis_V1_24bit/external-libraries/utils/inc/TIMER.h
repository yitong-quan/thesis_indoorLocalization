#ifndef DRIVERS_TIMER_H_
#define DRIVERS_TIMER_H_

#include "em_emu.h"
#include "em_cmu.h"
#include "em_timer.h"

volatile bool TIMER_TIMEOUT;
volatile bool TIMER_US_TIMEOUT;

/*
 *
 */
void TIMER_init(TIMER_TypeDef *timer, CMU_Clock_TypeDef timer_clock,
		IRQn_Type timer_irq);

/*
 *
 */
void TIMER_start(TIMER_TypeDef *timer, uint16_t timeout_ms);

/*
 *
 */
void TIMER_stop(TIMER_TypeDef *timer);

/*
 *
 */
void TIMER_delay_ms(TIMER_TypeDef *timer, uint16_t timeout_ms, bool use_EM1_mode);

/*
 *
 */
void TIMER_init_us(TIMER_TypeDef *timer, CMU_Clock_TypeDef timer_clock,
		IRQn_Type timer_irq);

/*
 *
 */
void TIMER_start_us(TIMER_TypeDef *timer, uint16_t timeout_us);

/*
 *
 */
void TIMER_stop_us(TIMER_TypeDef *timer);

/*
 *
 */
void TIMER_delay_us(TIMER_TypeDef *timer, uint16_t timeout_us);


/*
 *
 */
void TIMER0_IRQHandler(void);

/*
 *
 */
void TIMER2_IRQHandler(void);

#endif /* DRIVERS_TIMER_H_ */
