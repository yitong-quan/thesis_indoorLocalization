#include "EEPROM_handler.h"

void moveInterruptVectorToRam(void);

#if (REVISION == IDENTIFIER_REV_1) || (REVISION == IDENTIFIER_REV_2) || (REVISION == IDENTIFIER_REV_3) || (REVISION == TABLE_REV_1)
void EEPROM_init() {
	/* Move the interrupt vector table to RAM to safely handle interrupts
	 * while performing write/erase operations on flash */
	moveInterruptVectorToRam();

	/* Enables the flash controller for writing. */
	MSC_Init();

	/* Initialize the eeprom emulator using 3 pages. */
	if ( !EE_Init(3) ) {

		/* If the initialization fails we have to take some measure
		 * to obtain a valid set of pages. In this example we simply
		 * format the pages */
		EE_Format(3);
	}

	/* All variables should be declared prior to any writes. */
	EE_DeclareVariable(&ee_id_hi);
	EE_DeclareVariable(&ee_id_lo);
	EE_DeclareVariable(&ee_wu_id_hi);
	EE_DeclareVariable(&ee_wu_id_lo);
	EE_DeclareVariable(&ee_aes_0);
	EE_DeclareVariable(&ee_aes_1);
	EE_DeclareVariable(&ee_aes_2);
	EE_DeclareVariable(&ee_aes_3);
	EE_DeclareVariable(&ee_aes_4);
	EE_DeclareVariable(&ee_aes_5);
	EE_DeclareVariable(&ee_aes_6);
	EE_DeclareVariable(&ee_aes_7);
	EE_DeclareVariable(&ee_aes_8);
	EE_DeclareVariable(&ee_aes_9);
	EE_DeclareVariable(&ee_aes_10);
	EE_DeclareVariable(&ee_aes_11);
	EE_DeclareVariable(&ee_aes_12);
	EE_DeclareVariable(&ee_aes_13);
	EE_DeclareVariable(&ee_aes_14);
	EE_DeclareVariable(&ee_aes_15);
}

void EEPROM_write(uint32_t *ID, uint32_t *WU_ID, uint8_t *AES_KEY) {
	EE_Write(&ee_id_hi, (uint16_t) (*ID >> 16));
	EE_Write(&ee_id_lo, (uint16_t) (*ID & 0xFFFF));
	EE_Write(&ee_wu_id_hi, (uint16_t) (*WU_ID >> 16));
	EE_Write(&ee_wu_id_lo, (uint16_t) (*WU_ID & 0xFFFF));
	EE_Write(&ee_aes_0, AES_KEY[0]);
	EE_Write(&ee_aes_1, AES_KEY[1]);
	EE_Write(&ee_aes_2, AES_KEY[2]);
	EE_Write(&ee_aes_3, AES_KEY[3]);
	EE_Write(&ee_aes_4, AES_KEY[4]);
	EE_Write(&ee_aes_5, AES_KEY[5]);
	EE_Write(&ee_aes_6, AES_KEY[6]);
	EE_Write(&ee_aes_7, AES_KEY[7]);
	EE_Write(&ee_aes_8, AES_KEY[8]);
	EE_Write(&ee_aes_9, AES_KEY[9]);
	EE_Write(&ee_aes_10, AES_KEY[10]);
	EE_Write(&ee_aes_11, AES_KEY[11]);
	EE_Write(&ee_aes_12, AES_KEY[12]);
	EE_Write(&ee_aes_13, AES_KEY[13]);
	EE_Write(&ee_aes_14, AES_KEY[14]);
	EE_Write(&ee_aes_15, AES_KEY[15]);
}

void EEPROM_read_and_store_into(uint32_t *ID, uint32_t *WU_ID, uint8_t *AES_KEY) {
	// ID
	EE_Read(&ee_id_hi, &EEPROM_READ);
	*ID = (uint32_t) (EEPROM_READ << 16);
	EE_Read(&ee_id_lo, &EEPROM_READ);
	*ID |= (uint32_t) (EEPROM_READ);
	// WakeUp ID
	EE_Read(&ee_wu_id_hi, &EEPROM_READ);
	*WU_ID = (uint32_t) (EEPROM_READ << 16);
	EE_Read(&ee_wu_id_lo, &EEPROM_READ);
	*WU_ID |= (uint32_t) (EEPROM_READ);
	// AES KEY
	EE_Read(&ee_aes_0, &EEPROM_READ);
	AES_KEY[0] = EEPROM_READ;
	EE_Read(&ee_aes_1, &EEPROM_READ);
	AES_KEY[1] = EEPROM_READ;
	EE_Read(&ee_aes_2, &EEPROM_READ);
	AES_KEY[2] = EEPROM_READ;
	EE_Read(&ee_aes_3, &EEPROM_READ);
	AES_KEY[3] = EEPROM_READ;
	EE_Read(&ee_aes_4, &EEPROM_READ);
	AES_KEY[4] = EEPROM_READ;
	EE_Read(&ee_aes_5, &EEPROM_READ);
	AES_KEY[5] = EEPROM_READ;
	EE_Read(&ee_aes_6, &EEPROM_READ);
	AES_KEY[6] = EEPROM_READ;
	EE_Read(&ee_aes_7, &EEPROM_READ);
	AES_KEY[7] = EEPROM_READ;
	EE_Read(&ee_aes_8, &EEPROM_READ);
	AES_KEY[8] = EEPROM_READ;
	EE_Read(&ee_aes_9, &EEPROM_READ);
	AES_KEY[9] = EEPROM_READ;
	EE_Read(&ee_aes_10, &EEPROM_READ);
	AES_KEY[10] = EEPROM_READ;
	EE_Read(&ee_aes_11, &EEPROM_READ);
	AES_KEY[11] = EEPROM_READ;
	EE_Read(&ee_aes_12, &EEPROM_READ);
	AES_KEY[12] = EEPROM_READ;
	EE_Read(&ee_aes_13, &EEPROM_READ);
	AES_KEY[13] = EEPROM_READ;
	EE_Read(&ee_aes_14, &EEPROM_READ);
	AES_KEY[14] = EEPROM_READ;
	EE_Read(&ee_aes_15, &EEPROM_READ);
	AES_KEY[15] = EEPROM_READ;
}
#elif	(REVISION == BASE_REV_1) || (REVISION == BASE_REV_2)
void EEPROM_init() {
	/* Move the interrupt vector table to RAM to safely handle interrupts
	 * while performing write/erase operations on flash */
	moveInterruptVectorToRam();

	/* Enables the flash controller for writing. */
	MSC_Init();

	/* Initialize the eeprom emulator using 3 pages. */
	if (!EE_Init(4)) {

		/* If the initialization fails we have to take some measure
		 * to obtain a valid set of pages. In this example we simply
		 * format the pages */
		EE_Format(4);
	}

	/* All variables should be declared prior to any writes. */
	EE_DeclareVariable(&ip1);
	EE_DeclareVariable(&ip2);
	EE_DeclareVariable(&ip3);
	EE_DeclareVariable(&ip4);
	EE_DeclareVariable(&sub1);
	EE_DeclareVariable(&sub2);
	EE_DeclareVariable(&sub3);
	EE_DeclareVariable(&sub4);
	EE_DeclareVariable(&gw1);
	EE_DeclareVariable(&gw2);
	EE_DeclareVariable(&gw3);
	EE_DeclareVariable(&gw4);
	EE_DeclareVariable(&srvport);
	EE_DeclareVariable(&base_id_hi);
	EE_DeclareVariable(&base_id_lo);
	EE_DeclareVariable(&dhcp);
	EE_DeclareVariable(&ee_aes_0);
	EE_DeclareVariable(&ee_aes_1);
	EE_DeclareVariable(&ee_aes_2);
	EE_DeclareVariable(&ee_aes_3);
	EE_DeclareVariable(&ee_aes_4);
	EE_DeclareVariable(&ee_aes_5);
	EE_DeclareVariable(&ee_aes_6);
	EE_DeclareVariable(&ee_aes_7);
	EE_DeclareVariable(&ee_aes_8);
	EE_DeclareVariable(&ee_aes_9);
	EE_DeclareVariable(&ee_aes_10);
	EE_DeclareVariable(&ee_aes_11);
	EE_DeclareVariable(&ee_aes_12);
	EE_DeclareVariable(&ee_aes_13);
	EE_DeclareVariable(&ee_aes_14);
	EE_DeclareVariable(&ee_aes_15);
}
void EEPROM_read_and_store_into(uint32_t *ID, uint8_t *AES_KEY, uint8_t *IP,
		uint8_t *SM, uint8_t *GW, uint16_t *tcpPort, uint8_t *useDHCP) {
	// ID
	EE_Read(&base_id_hi, &EEPROM_READ);
	*ID = (uint32_t) (EEPROM_READ << 16);
	EE_Read(&base_id_lo, &EEPROM_READ);
	*ID |= (uint32_t) (EEPROM_READ);
	// Check if DHCP is used
	EE_Read(&dhcp, &EEPROM_READ);
	*useDHCP = (uint8_t) (EEPROM_READ & 0xFF);
	// TCP Server Port
	EE_Read(&srvport, &EEPROM_READ);
	*tcpPort = EEPROM_READ;
	if (*useDHCP == 0) {
		// Reads the IP from flash
		EE_Read(&ip1, &EEPROM_READ);
		IP[0] = EEPROM_READ;
		EE_Read(&ip2, &EEPROM_READ);
		IP[1] = EEPROM_READ;
		EE_Read(&ip3, &EEPROM_READ);
		IP[2] = EEPROM_READ;
		EE_Read(&ip4, &EEPROM_READ);
		IP[3] = EEPROM_READ;

		// Reads the Subnet Mask from flash
		EE_Read(&sub1, &EEPROM_READ);
		SM[0] = EEPROM_READ;
		EE_Read(&sub2, &EEPROM_READ);
		SM[1] = EEPROM_READ;
		EE_Read(&sub3, &EEPROM_READ);
		SM[2] = EEPROM_READ;
		EE_Read(&sub4, &EEPROM_READ);
		SM[3] = EEPROM_READ;

		// Reads the GW from flash
		EE_Read(&gw1, &EEPROM_READ);
		GW[0] = EEPROM_READ;
		EE_Read(&gw2, &EEPROM_READ);
		GW[1] = EEPROM_READ;
		EE_Read(&gw3, &EEPROM_READ);
		GW[2] = EEPROM_READ;
		EE_Read(&gw4, &EEPROM_READ);
		GW[3] = EEPROM_READ;
	}
	// Reads the AESKey from Flash
	EE_Read(&ee_aes_0, &EEPROM_READ);
	AES_KEY[0] = EEPROM_READ;
	EE_Read(&ee_aes_1, &EEPROM_READ);
	AES_KEY[1] = EEPROM_READ;
	EE_Read(&ee_aes_2, &EEPROM_READ);
	AES_KEY[2] = EEPROM_READ;
	EE_Read(&ee_aes_3, &EEPROM_READ);
	AES_KEY[3] = EEPROM_READ;
	EE_Read(&ee_aes_4, &EEPROM_READ);
	AES_KEY[4] = EEPROM_READ;
	EE_Read(&ee_aes_5, &EEPROM_READ);
	AES_KEY[5] = EEPROM_READ;
	EE_Read(&ee_aes_6, &EEPROM_READ);
	AES_KEY[6] = EEPROM_READ;
	EE_Read(&ee_aes_7, &EEPROM_READ);
	AES_KEY[7] = EEPROM_READ;
	EE_Read(&ee_aes_8, &EEPROM_READ);
	AES_KEY[8] = EEPROM_READ;
	EE_Read(&ee_aes_9, &EEPROM_READ);
	AES_KEY[9] = EEPROM_READ;
	EE_Read(&ee_aes_10, &EEPROM_READ);
	AES_KEY[10] = EEPROM_READ;
	EE_Read(&ee_aes_11, &EEPROM_READ);
	AES_KEY[11] = EEPROM_READ;
	EE_Read(&ee_aes_12, &EEPROM_READ);
	AES_KEY[12] = EEPROM_READ;
	EE_Read(&ee_aes_13, &EEPROM_READ);
	AES_KEY[13] = EEPROM_READ;
	EE_Read(&ee_aes_14, &EEPROM_READ);
	AES_KEY[14] = EEPROM_READ;
	EE_Read(&ee_aes_15, &EEPROM_READ);
	AES_KEY[15] = EEPROM_READ;
}

void EEPROM_write_id(uint32_t *ID) {
	EE_Write(&base_id_hi, (uint16_t) (*ID >> 16));
	EE_Write(&base_id_lo, (uint16_t) (*ID & 0xFFFF));
}
void EEPROM_write_aes(uint8_t *AES_KEY) {
	EE_Write(&ee_aes_0, AES_KEY[0]);
	EE_Write(&ee_aes_1, AES_KEY[1]);
	EE_Write(&ee_aes_2, AES_KEY[2]);
	EE_Write(&ee_aes_3, AES_KEY[3]);
	EE_Write(&ee_aes_4, AES_KEY[4]);
	EE_Write(&ee_aes_5, AES_KEY[5]);
	EE_Write(&ee_aes_6, AES_KEY[6]);
	EE_Write(&ee_aes_7, AES_KEY[7]);
	EE_Write(&ee_aes_8, AES_KEY[8]);
	EE_Write(&ee_aes_9, AES_KEY[9]);
	EE_Write(&ee_aes_10, AES_KEY[10]);
	EE_Write(&ee_aes_11, AES_KEY[11]);
	EE_Variable_TypeDef address;
	address.virtualAddress = 29;
	EE_Write(&address, AES_KEY[12]);
	EE_Write(&ee_aes_13, AES_KEY[13]);
	EE_Write(&ee_aes_14, AES_KEY[14]);
	EE_Write(&ee_aes_15, AES_KEY[15]);
}
void EEPROM_write_ip(uint8_t *IP) {
	EE_Write(&ip1, IP[0]);
	EE_Write(&ip2, IP[1]);
	EE_Write(&ip3, IP[2]);
	EE_Write(&ip4, IP[3]);
}
void EEPROM_write_subnetmask(uint8_t *SM) {
	EE_Write(&sub1, SM[0]);
	EE_Write(&sub2, SM[1]);
	EE_Write(&sub3, SM[2]);
	EE_Write(&sub4, SM[3]);
}
void EEPROM_write_gateway(uint8_t *GW) {
	EE_Write(&gw1, GW[0]);
	EE_Write(&gw2, GW[1]);
	EE_Write(&gw3, GW[2]);
	EE_Write(&gw4, GW[3]);
}
void EEPROM_write_tcpport(uint16_t tcpPort) {
	EE_Write(&srvport, tcpPort);
}

void EEPROM_write_dhcp(uint8_t useDHCP) {
	EE_Write(&dhcp, useDHCP);
}

#endif
