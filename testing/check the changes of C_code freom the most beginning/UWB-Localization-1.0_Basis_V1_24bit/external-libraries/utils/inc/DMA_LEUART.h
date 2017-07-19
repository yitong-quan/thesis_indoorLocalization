#ifndef UTILS_DMA_LEUART_H_
#define UTILS_DMA_LEUART_H_

#include "config.h"
#include "em_dma.h"
#include "em_leuart.h"

#define DMA_CHANNEL    0
#define BUF_MAX        50  // TODO: 1023
char DMA_LEUART_RX_BUF[BUF_MAX];
uint8_t BROADCAST_PATTERN;
uint32_t leuartif;
uint32_t DMA_len;

/*
 *
 */
void DMA_setup_leuart(void);

/* DMA control block, must be aligned to 256. */
#if defined (__ICCARM__)
#pragma data_alignment=256
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
#error Undefined toolkit, need to define alignment
#endif


#endif /* UTILS_DMA_LEUART_H_ */
