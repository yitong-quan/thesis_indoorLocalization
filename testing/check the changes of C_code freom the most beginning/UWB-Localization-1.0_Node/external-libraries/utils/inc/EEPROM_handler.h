#ifndef __EEPROM_HANDLER_H
#define __EEPROM_HANDLER_H

#include "config.h"
#include "revisionDef.h"

#include "EEPROM_DRV.h"

uint16_t EEPROM_READ;
#if (REVISION == IDENTIFIER_REV_1) || (REVISION == IDENTIFIER_REV_2) || (REVISION == IDENTIFIER_REV_3) || (REVISION == TABLE_REV_1)
/* EEPROM VALUES */
EE_Variable_TypeDef ee_id_hi, ee_id_lo, ee_wu_id_hi, ee_wu_id_lo, ee_aes_0, ee_aes_1, ee_aes_2,
	ee_aes_3, ee_aes_4, ee_aes_5, ee_aes_6, ee_aes_7, ee_aes_8, ee_aes_9, ee_aes_10, ee_aes_11,
	ee_aes_12, ee_aes_13, ee_aes_14, ee_aes_15;

void EEPROM_init();
void EEPROM_write(uint32_t *ID, uint32_t *WU_ID, uint8_t *AES_KEY);
void EEPROM_read_and_store_into(uint32_t *ID, uint32_t *WU_ID, uint8_t *AES_KEY);

#elif	(REVISION == BASE_REV_1) || (REVISION == BASE_REV_2)

/* EEPROM VALUES BASESTATION */
EE_Variable_TypeDef ip1, ip2, ip3, ip4, sub1, sub2, sub3, sub4, gw1, gw2, gw3,
		gw4, srvport, base_id_hi, base_id_lo, dhcp, ee_aes_0, ee_aes_1, ee_aes_2,
		ee_aes_3, ee_aes_4, ee_aes_5, ee_aes_6, ee_aes_7, ee_aes_8, ee_aes_9, ee_aes_10, ee_aes_11,
		ee_aes_12, ee_aes_13, ee_aes_14, ee_aes_15;

//
void EEPROM_init();

//
void EEPROM_read_and_store_into(uint32_t *ID, uint8_t *AES_KEY, uint8_t *IP, uint8_t *SM, uint8_t *GW, uint16_t *tcpPort, uint8_t *useDHCP);

//
void EEPROM_write_id(uint32_t *ID);
void EEPROM_write_aes(uint8_t *AES_KEY);
void EEPROM_write_ip(uint8_t *IP);
void EEPROM_write_subnetmask(uint8_t *SM);
void EEPROM_write_gateway(uint8_t *GW);
void EEPROM_write_tcpport(uint16_t tcpPort);
void EEPROM_write_dhcp(uint8_t useDHCP);

#endif

#endif /* __EEPROM_HANDLER_H */
