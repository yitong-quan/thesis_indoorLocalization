#include <em_gpio.h>
#include "ANTENNA.h"

void ANTENNA_switch(uint8_t port) {
#if		(REVISION == IDENTIFIER_REV_1) || (REVISION == IDENTIFIER_REV_2) || (REVISION == TABLE_REV_1)

	if (port == ANTENNA_DEST_CC1101)
	GPIO_PinOutSet(ANTENNA_CTRL_PORT, ANTENNA_CTRL_PIN);
	else
	GPIO_PinOutClear(ANTENNA_CTRL_PORT, ANTENNA_CTRL_PIN);

#elif	REVISION == IDENTIFIER_REV_3

	if (port == ANTENNA_DEST_CC1101)
	GPIO_PinOutClear(ANTENNA_CTRL_PORT, ANTENNA_CTRL_PIN);
	else
	GPIO_PinOutSet(ANTENNA_CTRL_PORT, ANTENNA_CTRL_PIN);

#endif
}

void BASE_ANTENNA_switch(uint8_t skyDest, uint8_t peDest, bool ant) {
#if	REVISION == BASE_REV_2
	if (peDest == PE_DEST_CC1190) {
		GPIO_PinOutClear(PE_CTRL_PORT, PE_CTRL_PIN); // RFC to RF2
	} else {
		GPIO_PinOutSet(PE_CTRL_PORT, PE_CTRL_PIN); // RFC to RF1
	}

	if (skyDest == SKY_DEST_CC1190) {
		if (ant) {
			// TX to ANT 2, RX to ANT 1
			GPIO_PinOutSet(SKY_V1_CTRL_PORT, SKY_V1_CTRL_PIN);
			GPIO_PinOutClear(SKY_V2_CTRL_PORT, SKY_V2_CTRL_PIN);
		} else {
			// TX to ANT 1, RX to ANT 2
			GPIO_PinOutClear(SKY_V1_CTRL_PORT, SKY_V1_CTRL_PIN);
			GPIO_PinOutSet(SKY_V2_CTRL_PORT, SKY_V2_CTRL_PIN);
		}
	} else {
		if (ant) {
			// TX to ANT 1, RX to ANT 2
			GPIO_PinOutClear(SKY_V1_CTRL_PORT, SKY_V1_CTRL_PIN);
			GPIO_PinOutSet(SKY_V2_CTRL_PORT, SKY_V2_CTRL_PIN);
		} else {
			// TX to ANT 2, RX to ANT 1
			GPIO_PinOutSet(SKY_V1_CTRL_PORT, SKY_V1_CTRL_PIN);
			GPIO_PinOutClear(SKY_V2_CTRL_PORT, SKY_V2_CTRL_PIN);
		}
	}
#endif
}
