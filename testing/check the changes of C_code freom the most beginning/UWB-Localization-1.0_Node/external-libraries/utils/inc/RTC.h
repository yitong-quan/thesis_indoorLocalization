#ifndef UTILS_RTC_H_
#define UTILS_RTC_H_

#include "em_rtc.h"
#include "em_emu.h"

bool RTC_TIMEOUT;

/*
 *
 */
void RTC_init(uint16_t timeout_ms);

/*
 *
 */
void RTC_start(uint16_t timeout_ms);

/*
 *
 */
void RTC_start_us(uint16_t timeout_us);

/*
 *
 */
void RTC_reset();

/*
 *
 */
void RTC_delay_us(uint16_t delay_us);

/*
 *
 */
void RTC_delay_ms(uint16_t delay_ms);

/*
 *
 */
void RTC_IRQHandler(void);

#endif /* UTILS_RTC_H_ */
