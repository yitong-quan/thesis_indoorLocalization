#include "CC1190.h"

#ifdef CC1190_PA_PORT
#if	REVISION == BASE_REV_1

	// switch to receive mode
void cc1190_receive(void) {
	GPIO_PinOutSet(CC1190_HGM_PORT, CC1190_HGM_PIN); // HighGainMode enable
	GPIO_PinOutSet(CC1190_LNA_PORT, CC1190_LNA_PIN); // LowNoiseAmp enable
	GPIO_PinOutClear(CC1190_PA_PORT, CC1190_PA_PIN); // PA disable
}

// switch to send mode
void cc1190_send(void) {
	GPIO_PinOutSet(CC1190_HGM_PORT, CC1190_HGM_PIN); // HighGainMode enable
	GPIO_PinOutClear(CC1190_LNA_PORT, CC1190_LNA_PIN); // LowNoiseAmp disable
	GPIO_PinOutSet(CC1190_PA_PORT, CC1190_PA_PIN); // PA enable
}

#elif	REVISION == BASE_REV_2

// switch to receive mode
void cc1190_receive(void) {
	GPIO_PinOutClear(CC1190_PA_PORT, CC1190_PA_PIN); // PA disable
}

// switch to send mode
void cc1190_send(void) {
	GPIO_PinOutSet(CC1190_PA_PORT, CC1190_PA_PIN); // PA enable
}

#endif
#endif /* def. CC1190_PA_PORT*/
