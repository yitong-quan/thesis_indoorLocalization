#include "CRC.h"

/*--------------------------------------------------
Function      : AddToCrc16
Parameter     : -
Return values : -
Purpose       : Adds a byte to CRC16
--------------------------------------------------*/
uint16_t AddToCrc16(uint16_t crc, uint8_t c)
{
  // CCITT 16 bit (X^16 + X^12 + X^5 + 1).
  crc  = (uint8_t)(crc >> 8) | (uint16_t)(crc << 8);
  crc ^= c;
  crc ^= (uint8_t)(crc & 0xff) >> 4;
  crc ^= (uint16_t) (((uint16_t)(crc << 8)) << 4);
  crc ^= ((uint16_t)((crc & 0xff) << 4)) << 1;
  return (crc);
}

/*--------------------------------------------------
Function      : CalcBlockCrc
Parameter     : ptr to memory
                len of block
Return values : CRC16
Purpose       : calculates a crc over a memory block
--------------------------------------------------*/
uint16_t CalcBlockCrc(uint8_t *pSrc, const uint32_t len)
{
  uint8_t *pMem;
  uint16_t crc;

  crc = 0;
  for (pMem = pSrc; pMem <= (uint8_t *) ((uint32_t) pSrc + len - 1); pMem++)
  {
    crc = AddToCrc16(crc, *pMem);
  }
  return crc;
}

//
uint16_t crc16(uint8_t *bytes, uint8_t start_idx, uint8_t end_idx) {
    uint8_t i, len;
    int crc_value = 0;
    for (len = start_idx; len < end_idx; len++) {
        for (i = 0x80; i != 0; i >>= 1) {
            if ((crc_value & 0x8000) != 0) {
                crc_value = (crc_value << 1) ^ 0x8005;
            } else {
                crc_value = crc_value << 1;
            }
            if ((bytes[len] & i) != 0) {
                crc_value ^= 0x8005;
            }
        }
    }
    return crc_value;
}
