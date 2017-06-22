#ifndef UTILS_CRC_H_
#define UTILS_CRC_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

extern uint16_t AddToCrc16(uint16_t crc, uint8_t c);
extern uint16_t CalcBlockCrc(uint8_t *pSrc, const uint32_t len);
uint16_t crc16(uint8_t *bytes, uint8_t start_idx, uint8_t end_idx);

#endif /* UTILS_CRC_H_ */
