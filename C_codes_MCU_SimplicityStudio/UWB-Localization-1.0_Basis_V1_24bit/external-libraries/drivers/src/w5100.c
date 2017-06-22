#include "w5100.h"

#ifdef W5100IntPort

#define TX_RX_MAX_BUF_SIZE 2048
#define TX_BUF 0x1100
#define RX_BUF (TX_BUF + TX_RX_MAX_BUF_SIZE)

#define TXBUF_BASE 0x4000
#define RXBUF_BASE 0x6000

void W5100_init(void) {
	//Delay(250);

	W5100_writeMR(1 << RST);
	W5100_writeTMSR(0x55);
	W5100_writeRMSR(0x55);
	W5100_writeIMR(0xEF);

	for (int i = 0; i < MAX_SOCK_NUM; i++) {
		SBASE[i] = TXBUF_BASE + SSIZE * i;
		RBASE[i] = RXBUF_BASE + RSIZE * i;
	}
}

uint16_t W5100_getTXFreeSize(SOCKET s) {
	uint16_t val = 0, val1 = 0;
	do {
		val1 = W5100_readSnTX_FSR(s);
		if (val1 != 0)
			val = W5100_readSnTX_FSR(s);
	} while (val != val1);
	return val;
}

uint16_t W5100_getRXReceivedSize(SOCKET s) {
	uint16_t val = 0, val1 = 0;
	do {
		val1 = W5100_readSnRX_RSR(s);
		if (val1 != 0)
			val = W5100_readSnRX_RSR(s);
	} while (val != val1);
	return val;
}

void W5100_send_data_processing(SOCKET s, const uint8_t *data, uint16_t len) {
	// This is same as having no offset in a call to send_data_processing_offset
	W5100_send_data_processing_offset(s, 0, data, len);
}

void W5100_send_data_processing_offset(SOCKET s, uint16_t data_offset,
		const uint8_t *data, uint16_t len) {
	uint16_t ptr = W5100_readSnTX_WR(s);
	ptr += data_offset;
	uint16_t offset = ptr & SMASK;
	uint16_t dstAddr = offset + SBASE[s];

	if (offset + len > SSIZE) {
		// Wrap around circular buffer
		uint16_t size = SSIZE - offset;
		W5100_write16(dstAddr, data, size);
		W5100_write16(SBASE[s], data + size, len - size);
	} else {
		W5100_write16(dstAddr, data, len);
	}

	ptr += len;
	W5100_writeSnTX_WR(s, ptr);
}

void W5100_recv_data_processing(SOCKET s, uint8_t *data, uint16_t len,
		uint8_t peek) {
	peek = 0;
	uint16_t ptr;
	ptr = W5100_readSnRX_RD(s);
	W5100_read_data(s, ptr, data, len);
	if (!peek) {
		ptr += len;
		W5100_writeSnRX_RD(s, ptr);
	}
}

void W5100_read_data(SOCKET s, volatile uint16_t src, volatile uint8_t *dst,
		uint16_t len) {
	uint16_t size;
	uint16_t src_mask;
	uint16_t src_ptr;

	src_mask = src & RMASK;
	src_ptr = RBASE[s] + src_mask;

	if ((src_mask + len) > RSIZE) {
		size = RSIZE - src_mask;
		W5100_read16(src_ptr, (uint8_t *) dst, size);
		dst += size;
		W5100_read16(RBASE[s], (uint8_t *) dst, len - size);
	} else
		W5100_read16(src_ptr, (uint8_t *) dst, len);
}

uint8_t W5100_write8(uint16_t _addr, uint8_t _data) {
	//setSS
	GPIO_PinOutClear(CSPort, CSPin);
#if		REVISION == BASE_REV_1
	GPIO_PinOutSet(SENPort, SENPin);
#endif
	// Fixme for debugging
	GPIO_PinOutClear(CSEPort, CSEPin);
	SPI_WriteByte(0xF0);
	SPI_WriteByte(_addr >> 8);
	SPI_WriteByte(_addr & 0xFF);
	SPI_WriteByte(_data);
	//resetSS
	GPIO_PinOutSet(CSPort, CSPin);
#if		REVISION == BASE_REV_1
	GPIO_PinOutClear(SENPort, SENPin);
#endif
	// Fixme for debugging
	GPIO_PinOutSet(CSEPort, CSEPin);
	return 1;
}

uint16_t W5100_write16(uint16_t _addr, const uint8_t *_buf, uint16_t _len) {

	for (uint16_t i = 0; i < _len; i++) {
		//setSS
#if		REVISION == BASE_REV_1
		GPIO_PinOutSet(SENPort, SENPin);
#endif
		GPIO_PinOutClear(CSPort, CSPin);
		// Fixme for debugging
		GPIO_PinOutClear(CSEPort, CSEPin);
		SPI_WriteByte(0xF0);
		SPI_WriteByte(_addr >> 8);
		SPI_WriteByte(_addr & 0xFF);
		_addr++;
		SPI_WriteByte(_buf[i]);
		//resetSS
		GPIO_PinOutSet(CSPort, CSPin);
#if		REVISION == BASE_REV_1
		GPIO_PinOutClear(SENPort, SENPin);
#endif
		// Fixme for debugging
		GPIO_PinOutSet(CSEPort, CSEPin);

	}

	return _len;
}

uint8_t W5100_read8(uint16_t _addr) {
	//setSS
#if		REVISION == BASE_REV_1
	GPIO_PinOutSet(SENPort, SENPin);
#endif
	GPIO_PinOutClear(CSPort, CSPin);
	// Fixme for debugging
	GPIO_PinOutClear(CSEPort, CSEPin);
	SPI_WriteByte(0x0F);
	SPI_WriteByte(_addr >> 8);
	SPI_WriteByte(_addr & 0xFF);
	uint8_t _data = SPI_ReadByte();
	//resetSS
	GPIO_PinOutSet(CSPort, CSPin);
#if		REVISION == BASE_REV_1
	GPIO_PinOutClear(SENPort, SENPin);
#endif
	// Fixme for debugging
	GPIO_PinOutSet(CSEPort, CSEPin);
	return _data;
}

uint16_t W5100_read16(uint16_t _addr, uint8_t *_buf, uint16_t _len) {

	for (uint16_t i = 0; i < _len; i++) {
		//setSS
#if		REVISION == BASE_REV_1
		GPIO_PinOutSet(SENPort, SENPin);
#endif
		GPIO_PinOutClear(CSPort, CSPin);
		// Fixme for debugging
		GPIO_PinOutClear(CSEPort, CSEPin);
		SPI_WriteByte(0x0F);
		SPI_WriteByte(_addr >> 8);
		SPI_WriteByte(_addr & 0xFF);
		_addr++;
		_buf[i] = SPI_ReadByte();
		//resetSS
		GPIO_PinOutSet(CSPort, CSPin);
#if		REVISION == BASE_REV_1
		GPIO_PinOutClear(SENPort, SENPin);
#endif
		// Fixme for debugging
		GPIO_PinOutSet(CSEPort, CSEPin);
	}

	return _len;
}

void W5100_execCmdSn(SOCKET s, SockCMD _cmd) {
	// Send command to socket
	W5100_writeSnCR(s, _cmd);
	// Wait for command to complete
	while (W5100_readSnCR(s))
		;
}

void W5100_getGatewayIp(uint8_t *_addr) {
	W5100_readGAR(_addr);
}

void W5100_setGatewayIp(uint8_t *_addr) {
	W5100_writeGAR(_addr);
}

void W5100_getSubnetMask(uint8_t *_addr) {
	W5100_readSUBR(_addr);
}

void W5100_setSubnetMask(uint8_t *_addr) {
	W5100_writeSUBR(_addr);
}

void W5100_getMACAddress(uint8_t *_addr) {
	W5100_readSHAR(_addr);
}

void W5100_setMACAddress(uint8_t *_addr) {
	W5100_writeSHAR(_addr);
}

void W5100_getIPAddress(uint8_t *_addr) {
	W5100_readSIPR(_addr);
}

void W5100_setIPAddress(uint8_t *_addr) {
	W5100_writeSIPR(_addr);
}

void W5100_setRetransmissionTime(uint16_t _timeout) {
	W5100_writeRTR(_timeout);
}

void W5100_setRetransmissionCount(uint8_t _retry) {
	W5100_writeRCR(_retry);
}

// This function writes a byte to the TX buffer (waits for buffer to clear)
void SPI_WriteByte(uint8_t byte) {
	USART_SpiTransfer(USART0, byte);
}

// This function returns the value of the RX buffer (waits for valid RX data)
uint8_t SPI_ReadByte(void) {
	return USART_SpiTransfer(USART0, 0x00);
}

#endif /* def. W5100IntPort */
