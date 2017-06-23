//=========================================================
// inc/InitDevice.h: generated by Hardware Configurator
//
// This file will be regenerated when saving a document.
// leave the sections inside the "$[...]" comment tags alone
// or they will be overwritten!
//=========================================================
#ifndef __INIT_DEVICE_H__
#define __INIT_DEVICE_H__

#include "config.h"
#include "revisionDef.h"

#include "em_system.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_adc.h"
#include "em_wdog.h"
#include "em_aes.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include "em_rtc.h"
#include "em_timer.h"
#include "em_usart.h"

#if		REVISION == IDENTIFIER_REV_1

	// USER CONSTANTS
// USER PROTOTYPES

// $[Mode Transition Prototypes]
extern void enter_DefaultMode_from_RESET(void);
// [Mode Transition Prototypes]$

// $[Config(Per-Module Mode)Transition Prototypes]
extern void HFXO_enter_DefaultMode_from_RESET(void);
extern void LFXO_enter_DefaultMode_from_RESET(void);
extern void CMU_enter_DefaultMode_from_RESET(void);
extern void ADC0_enter_DefaultMode_from_RESET(void);
extern void ACMP0_enter_DefaultMode_from_RESET(void);
extern void IDAC0_enter_DefaultMode_from_RESET(void);
extern void RTC_enter_DefaultMode_from_RESET(void);
extern void USART0_enter_DefaultMode_from_RESET(void);
extern void USART1_enter_DefaultMode_from_RESET(void);
extern void LEUART0_enter_DefaultMode_from_RESET(void);
extern void VCMP_enter_DefaultMode_from_RESET(void);
extern void WDOG_enter_DefaultMode_from_RESET(void);
extern void I2C0_enter_DefaultMode_from_RESET(void);
extern void TIMER0_enter_DefaultMode_from_RESET(void);
extern void TIMER1_enter_DefaultMode_from_RESET(void);
extern void PCNT0_enter_DefaultMode_from_RESET(void);
extern void PRS_enter_DefaultMode_from_RESET(void);
extern void PORTIO_enter_DefaultMode_from_RESET(void);
// [Config(Per-Module Mode)Transition Prototypes]$

#elif	REVISION == IDENTIFIER_REV_2

	// USER CONSTANTS
	// USER PROTOTYPES

	// $[Mode Transition Prototypes]
	extern void enter_DefaultMode_from_RESET(void);
	// [Mode Transition Prototypes]$

	// $[Config(Per-Module Mode)Transition Prototypes]
	extern void HFXO_enter_DefaultMode_from_RESET(void);
	extern void LFXO_enter_DefaultMode_from_RESET(void);
	extern void CMU_enter_DefaultMode_from_RESET(void);
	extern void ADC0_enter_DefaultMode_from_RESET(void);
	extern void ACMP0_enter_DefaultMode_from_RESET(void);
	extern void IDAC0_enter_DefaultMode_from_RESET(void);
	extern void RTC_enter_DefaultMode_from_RESET(void);
	extern void USART0_enter_DefaultMode_from_RESET(void);
	extern void USART1_enter_DefaultMode_from_RESET(void);
	extern void LEUART0_enter_DefaultMode_from_RESET(void);
	extern void VCMP_enter_DefaultMode_from_RESET(void);
	extern void WDOG_enter_DefaultMode_from_RESET(void);
	extern void I2C0_enter_DefaultMode_from_RESET(void);
	extern void TIMER0_enter_DefaultMode_from_RESET(void);
	extern void TIMER1_enter_DefaultMode_from_RESET(void);
	extern void PCNT0_enter_DefaultMode_from_RESET(void);
	extern void PRS_enter_DefaultMode_from_RESET(void);
	extern void PORTIO_enter_DefaultMode_from_RESET(void);
	// [Config(Per-Module Mode)Transition Prototypes]$

#elif	REVISION == IDENTIFIER_REV_3

	// USER CONSTANTS
	// USER PROTOTYPES

	// $[Mode Transition Prototypes]
	extern void enter_DefaultMode_from_RESET(void);
	// [Mode Transition Prototypes]$

	// $[Config(Per-Module Mode)Transition Prototypes]
	extern void HFXO_enter_DefaultMode_from_RESET(void);
	extern void LFXO_enter_DefaultMode_from_RESET(void);
	extern void CMU_enter_DefaultMode_from_RESET(void);
	extern void ADC0_enter_DefaultMode_from_RESET(void);
	extern void ACMP0_enter_DefaultMode_from_RESET(void);
	extern void IDAC0_enter_DefaultMode_from_RESET(void);
	extern void RTC_enter_DefaultMode_from_RESET(void);
	extern void USART0_enter_DefaultMode_from_RESET(void);
	extern void USART1_enter_DefaultMode_from_RESET(void);
	extern void LEUART0_enter_DefaultMode_from_RESET(void);
	extern void VCMP_enter_DefaultMode_from_RESET(void);
	extern void WDOG_enter_DefaultMode_from_RESET(void);
	extern void I2C0_enter_DefaultMode_from_RESET(void);
	extern void TIMER0_enter_DefaultMode_from_RESET(void);
	extern void TIMER1_enter_DefaultMode_from_RESET(void);
	extern void PCNT0_enter_DefaultMode_from_RESET(void);
	extern void PRS_enter_DefaultMode_from_RESET(void);
	extern void PORTIO_enter_DefaultMode_from_RESET(void);
	// [Config(Per-Module Mode)Transition Prototypes]$

#elif	REVISION == TABLE_REV_1

	// USER CONSTANTS
	// USER PROTOTYPES

	// $[Mode Transition Prototypes]
	extern void enter_DefaultMode_from_RESET(void);
	// [Mode Transition Prototypes]$

	// $[Config(Per-Module Mode)Transition Prototypes]
	extern void HFXO_enter_DefaultMode_from_RESET(void);
	extern void LFXO_enter_DefaultMode_from_RESET(void);
	extern void CMU_enter_DefaultMode_from_RESET(void);
	extern void ADC0_enter_DefaultMode_from_RESET(void);
	extern void ACMP0_enter_DefaultMode_from_RESET(void);
	extern void IDAC0_enter_DefaultMode_from_RESET(void);
	extern void RTC_enter_DefaultMode_from_RESET(void);
	extern void USART0_enter_DefaultMode_from_RESET(void);
	extern void USART1_enter_DefaultMode_from_RESET(void);
	extern void LEUART0_enter_DefaultMode_from_RESET(void);
	extern void VCMP_enter_DefaultMode_from_RESET(void);
	extern void WDOG_enter_DefaultMode_from_RESET(void);
	extern void I2C0_enter_DefaultMode_from_RESET(void);
	extern void TIMER0_enter_DefaultMode_from_RESET(void);
	extern void TIMER1_enter_DefaultMode_from_RESET(void);
	extern void PCNT0_enter_DefaultMode_from_RESET(void);
	extern void PRS_enter_DefaultMode_from_RESET(void);
	extern void PORTIO_enter_DefaultMode_from_RESET(void);
	// [Config(Per-Module Mode)Transition Prototypes]$

#elif	REVISION == BASE_REV_2

#endif

#endif
