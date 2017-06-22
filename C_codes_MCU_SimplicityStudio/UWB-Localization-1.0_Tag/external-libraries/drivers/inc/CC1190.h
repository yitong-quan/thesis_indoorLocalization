#ifndef DRIVERS_CC1190_H_
#define DRIVERS_CC1190_H_

#include "config.h"
#include "revisionDef.h"

#include "em_gpio.h"

#if	REVISION == BASE_REV_1

	#define CC1190_LNA_PORT	gpioPortC
	#define CC1190_LNA_PIN 	0

	#define CC1190_PA_PORT 	gpioPortC
	#define CC1190_PA_PIN 	1

	#define CC1190_HGM_PORT	gpioPortA
	#define CC1190_HGM_PIN 	2

#elif	REVISION == BASE_REV_2

	#define CC1190_PA_PORT 	gpioPortB
	#define CC1190_PA_PIN 	11

#endif

// switch to receive mode
void cc1190_receive (void);

// switch to send mode
void cc1190_send (void);

#endif /* DRIVERS_CC1190_H_ */

