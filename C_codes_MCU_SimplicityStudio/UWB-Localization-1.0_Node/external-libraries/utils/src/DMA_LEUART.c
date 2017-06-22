#include "DMA_LEUART.h"

/* DMA init structure */
DMA_Init_TypeDef dmaInit = {
		.hprot = 0, /* No descriptor protection */
		.controlBlock = dmaControlBlock, /* DMA control block aligned to 256 */
};

/* Setting up channel */
DMA_CfgChannel_TypeDef chnlCfg = {
		.highPri = false, /* Normal priority */
		.enableInt = false, /* No interrupt enabled for callback functions */
		.select = DMAREQ_LEUART0_RXDATAV, /* Set LEUART0 RX data available as source of DMA signals */
		.cb = NULL , /* No callback function */
};

/* Setting up channel descriptor */
DMA_CfgDescr_TypeDef descrCfg = {
			.dstInc = dmaDataInc1, /* Increment destination address by one byte */
			.srcInc = dmaDataIncNone, /* Do no increment source address  */
			.size = dmaDataSize1, /* Data size is one byte */
			.arbRate = dmaArbitrate1, /* Rearbitrate for each byte received*/
			.hprot = 0, /* No read/write source protection */
};

//
void DMA_setup_leuart(void) {
	memset(DMA_LEUART_RX_BUF, 0x00, BUF_MAX);
	/* Initializing DMA, channel and descriptor */
	DMA_Init(&dmaInit);
	DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);
	DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);

	/* Starting the transfer. Using Basic Mode */
	DMA_ActivateBasic(DMA_CHANNEL, /* Activate channel selected */
	true, /* Use primary descriptor */
	false, /* No DMA burst */
	(void *) &DMA_LEUART_RX_BUF, /* Destination address */
	(void *) &LEUART0 ->RXDATA, /* Source address*/
	BUF_MAX - 1); /* Size of buffer minus1 */

	/* Set LEUART signal frame */LEUART0 ->SIGFRAME = '\r';

	/* Enable LEUART Signal Frame Interrupt */
	LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF);

	/* Enable LEUART0 interrupt vector */
	NVIC_EnableIRQ(LEUART0_IRQn);

	/* Make sure the LEUART wakes up the DMA on RX data */
	LEUART0 ->CTRL |= LEUART_CTRL_RXDMAWU;
}

