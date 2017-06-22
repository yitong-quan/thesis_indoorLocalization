#ifndef DRIVERS_ANTENNA_H_
#define DRIVERS_ANTENNA_H_

#include "config.h"
#include "revisionDef.h"

extern bool antUse;


#if		(REVISION == IDENTIFIER_REV_1) || (REVISION == IDENTIFIER_REV_2) || (REVISION == IDENTIFIER_REV_3) || (REVISION == TABLE_REV_1)

	// Antenna switch Destinations:
	#define ANTENNA_DEST_CC1101		0
	#define ANTENNA_DEST_WAKEUPRF	1

	#define ANTENNA_CTRL_PORT		gpioPortA
	#define ANTENNA_CTRL_PIN		2

#elif	REVISION == BASE_REV_2

	#define ANT1					true
	#define ANT2					false

	// This is needed to prevent compiler errors at Basestation
	#define ANTENNA_DEST_CC1101		0
	#define ANTENNA_DEST_WAKEUPRF	1

	// Antenna switch Destinations:
	#define SKY_DEST_CC1190		0
	#define SKY_DEST_PE			1

	#define PE_DEST_CC1190		0
	#define PE_DEST_SKY			1

	#define SKY_V1_CTRL_PORT	gpioPortC
	#define SKY_V1_CTRL_PIN		0

	#define SKY_V2_CTRL_PORT	gpioPortC
	#define SKY_V2_CTRL_PIN		1

	#define PE_CTRL_PORT		gpioPortA
	#define PE_CTRL_PIN			2

#endif

	/**
	 * Switch antenna to CC1101 or WAKEUPRF
	 */
	void ANTENNA_switch(uint8_t dest);

	/**
	 * Switch antenna to CC1101 or WAKEUPRF
	 */
	void BASE_ANTENNA_switch(uint8_t skyDest, uint8_t peDest, bool ant);

#endif /* DRIVERS_ANTENNA_H_ */
