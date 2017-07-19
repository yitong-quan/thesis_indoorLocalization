#ifndef	W5100_H
#define	W5100_H

#include "config.h"
#include "revisionDef.h"

#include <stdint.h>
#include "em_gpio.h"
#include "em_usart.h"

#if		REVISION == BASE_REV_1

#define CSPort gpioPortE
#define CSPin 13

#define W5100IntPort gpioPortA
#define W5100IntPin 0

// Necesarry because W5100 needs SEN pin low otherwise the SPI-Interface is blocked
#define SENPort gpioPortF
#define SENPin 2

//Fixme for debugging
#define CSEPort gpioPortA
#define CSEPin 1

#elif	REVISION == BASE_REV_2

#define CSPort gpioPortE
#define CSPin 13

//Fixme for debugging
#define CSEPort gpioPortA
#define CSEPin 1

#define W5100IntPort gpioPortA
#define W5100IntPin 0

#endif

#define MAX_SOCK_NUM 4

typedef uint8_t SOCKET;

#define IDM_OR  0x8000
#define IDM_AR0 0x8001
#define IDM_AR1 0x8002
#define IDM_DR  0x8003

  static const uint8_t SnMR_CLOSE  = 0x00;
  static const uint8_t SnMR_TCP    = 0x01;
  static const uint8_t SnMR_UDP    = 0x02;
  static const uint8_t SnMR_IPRAW  = 0x03;
  static const uint8_t SnMR_MACRAW = 0x04;
  static const uint8_t SnMR_PPPOE  = 0x05;
  static const uint8_t SnMR_ND     = 0x20;
  static const uint8_t SnMR_MULTI  = 0x80;

enum SockCMD {
  Sock_OPEN      = 0x01,
  Sock_LISTEN    = 0x02,
  Sock_CONNECT   = 0x04,
  Sock_DISCON    = 0x08,
  Sock_CLOSE     = 0x10,
  Sock_SEND      = 0x20,
  Sock_SEND_MAC  = 0x21,
  Sock_SEND_KEEP = 0x22,
  Sock_RECV      = 0x40
};

typedef enum SockCMD SockCMD;

  static const uint8_t SnIR_SEND_OK = 0x10;
  static const uint8_t SnIR_TIMEOUT = 0x08;
  static const uint8_t SnIR_RECV    = 0x04;
  static const uint8_t SnIR_DISCON  = 0x02;
  static const uint8_t SnIR_CON     = 0x01;

  static const uint8_t SnSR_CLOSED      = 0x00;
  static const uint8_t SnSR_INIT        = 0x13;
  static const uint8_t SnSR_LISTEN      = 0x14;
  static const uint8_t SnSR_SYNSENT     = 0x15;
  static const uint8_t SnSR_SYNRECV     = 0x16;
  static const uint8_t SnSR_ESTABLISHED = 0x17;
  static const uint8_t SnSR_FIN_WAIT    = 0x18;
  static const uint8_t SnSR_CLOSING     = 0x1A;
  static const uint8_t SnSR_TIME_WAIT   = 0x1B;
  static const uint8_t SnSR_CLOSE_WAIT  = 0x1C;
  static const uint8_t SnSR_LAST_ACK    = 0x1D;
  static const uint8_t SnSR_UDP         = 0x22;
  static const uint8_t SnSR_IPRAW       = 0x32;
  static const uint8_t SnSR_MACRAW      = 0x42;
  static const uint8_t SnSR_PPPOE       = 0x5F;

  static const uint8_t IPPROTO_IP   = 0;
  static const uint8_t IPPROTO_ICMP = 1;
  static const uint8_t IPPROTO_IGMP = 2;
  static const uint8_t IPPROTO_GGP  = 3;
  static const uint8_t IPPROTO_TCP  = 6;
  static const uint8_t IPPROTO_PUP  = 12;
  static const uint8_t IPPROTO_UDP  = 17;
  static const uint8_t IPPROTO_IDP  = 22;
  static const uint8_t IPPROTO_ND   = 77;
  static const uint8_t IPPROTO_RAW  = 255;

static const uint8_t  RST = 7; // Reset BIT

#define SOCKETS 4
static const uint16_t SMASK = 0x07FF; // Tx buffer MASK
static const uint16_t RMASK = 0x07FF; // Rx buffer MASK
static const uint16_t SSIZE = 2048; // Max Tx buffer size
static const uint16_t RSIZE = 2048; // Max Rx buffer size
uint16_t SBASE[SOCKETS]; // Tx buffer base address
uint16_t RBASE[SOCKETS]; // Rx buffer base address

static const uint16_t CH_BASE = 0x0400;
static const uint16_t CH_SIZE = 0x0100;


void W5100_init();

  /**
   * @brief	This function is being used for copy the data form Receive buffer of the chip to application buffer.
   *
   * It calculate the actual physical address where one has to read
   * the data from Receive buffer. Here also take care of the condition while it exceed
   * the Rx memory uper-bound of socket.
   */
  void W5100_read_data(SOCKET s, volatile uint16_t src, volatile uint8_t * dst, uint16_t len);

  /**
   * @brief	 This function is being called by send() and sendto() function also.
   *
   * This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
   * register. User should read upper byte first and lower byte later to get proper value.
   */
  void W5100_send_data_processing(SOCKET s, const uint8_t *data, uint16_t len);
  /**
   * @brief A copy of send_data_processing that uses the provided ptr for the
   *        write offset.  Only needed for the "streaming" UDP API, where
   *        a single UDP packet is built up over a number of calls to
   *        send_data_processing_ptr, because TX_WR doesn't seem to get updated
   *        correctly in those scenarios
   * @param ptr value to use in place of TX_WR.  If 0, then the value is read
   *        in from TX_WR
   * @return New value for ptr, to be used in the next call
   */
// FIXME Update documentation
  void W5100_send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len);

  /**
   * @brief	This function is being called by recv() also.
   *
   * This function read the Rx read pointer register
   * and after copy the data from receive buffer update the Rx write pointer register.
   * User should read upper byte first and lower byte later to get proper value.
   */
  void W5100_recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek);

  uint16_t W5100_getTXFreeSize(SOCKET s);
  uint16_t W5100_getRXReceivedSize(SOCKET s);

uint8_t W5100_write8(uint16_t _addr, uint8_t _data);
uint16_t W5100_write16(uint16_t _addr, const uint8_t *_buf, uint16_t _len);
uint8_t W5100_read8(uint16_t _addr);
uint16_t W5100_read16(uint16_t _addr, uint8_t *_buf, uint16_t _len);

#define __GP_REGISTER8(name, address)             \
  static inline void W5100_write##name(uint8_t _data) { \
    W5100_write8(address, _data);                        \
  }                                               \
  static inline uint8_t W5100_read##name() {            \
    return W5100_read8(address);                         \
  }
#define __GP_REGISTER16(name, address)            \
  static void W5100_write##name(uint16_t _data) {       \
    W5100_write8(address,   _data >> 8);                 \
    W5100_write8(address+1, _data & 0xFF);               \
  }                                               \
  static uint16_t W5100_read##name() {                  \
    uint16_t res = W5100_read8(address);                 \
    res = (res << 8) + W5100_read8(address + 1);         \
    return res;                                   \
  }
#define __GP_REGISTER_N(name, address, size)      \
  static uint16_t W5100_write##name(uint8_t *_buff) {   \
    return W5100_write16(address, _buff, size);           \
  }                                               \
  static uint16_t W5100_read##name(uint8_t *_buff) {    \
    return W5100_read16(address, _buff, size);            \
  }

__GP_REGISTER8(MR, 0x0000);// Mode
__GP_REGISTER_N(GAR, 0x0001, 4);// Gateway IP address
__GP_REGISTER_N(SUBR, 0x0005, 4);// Subnet mask address
__GP_REGISTER_N(SHAR, 0x0009, 6);// Source MAC address
__GP_REGISTER_N(SIPR, 0x000F, 4);// Source IP address
__GP_REGISTER8(IR, 0x0015);// Interrupt
__GP_REGISTER8(IMR, 0x0016);// Interrupt Mask
__GP_REGISTER16(RTR, 0x0017);// Timeout address
__GP_REGISTER8(RCR, 0x0019);// Retry count
__GP_REGISTER8(RMSR, 0x001A);// Receive memory size
__GP_REGISTER8(TMSR, 0x001B);// Transmit memory size
__GP_REGISTER8(PATR, 0x001C);// Authentication type address in PPPoE mode
__GP_REGISTER8(PTIMER, 0x0028);// PPP LCP Request Timer
__GP_REGISTER8(PMAGIC, 0x0029);// PPP LCP Magic Number
__GP_REGISTER_N(UIPR, 0x002A, 4);// Unreachable IP address in UDP mode
__GP_REGISTER16(UPORT, 0x002E);// Unreachable Port address in UDP mode

#undef __GP_REGISTER8
#undef __GP_REGISTER16
#undef __GP_REGISTER_N

// W5100 Socket registers
// ----------------------
static inline uint8_t readSn8(SOCKET _s, uint16_t _addr);
static inline uint8_t writeSn8(SOCKET _s, uint16_t _addr, uint8_t _data);
static inline uint16_t readSn16(SOCKET _s, uint16_t _addr, uint8_t *_buf, uint16_t len);
static inline uint16_t writeSn16(SOCKET _s, uint16_t _addr, uint8_t *_buf, uint16_t len);


#define __SOCKET_REGISTER8(name, address)                    \
  static inline void W5100_write##name(SOCKET _s, uint8_t _data) { \
    writeSn8(_s, address, _data);                             \
  }                                                          \
  static inline uint8_t W5100_read##name(SOCKET _s) {              \
    return readSn8(_s, address);                              \
  }
#define __SOCKET_REGISTER16(name, address)                   \
  static void W5100_write##name(SOCKET _s, uint16_t _data) {       \
    writeSn8(_s, address,   _data >> 8);                      \
    writeSn8(_s, address+1, _data & 0xFF);                    \
  }                                                          \
  static uint16_t W5100_read##name(SOCKET _s) {                    \
    uint16_t res = readSn8(_s, address);                      \
    uint16_t res2 = readSn8(_s,address + 1);                  \
    res = res << 8;                                          \
    res2 = res2 & 0xFF;                                      \
    res = res | res2;                                        \
    return res;                                              \
  }
#define __SOCKET_REGISTER_N(name, address, size)             \
  static uint16_t W5100_write##name(SOCKET _s, uint8_t *_buff) {   \
    return writeSn16(_s, address, _buff, size);                \
  }                                                          \
  static uint16_t W5100_read##name(SOCKET _s, uint8_t *_buff) {    \
    return readSn16(_s, address, _buff, size);                 \
  }

__SOCKET_REGISTER8(SnMR, 0x0000)// Mode
__SOCKET_REGISTER8(SnCR, 0x0001)// Command
__SOCKET_REGISTER8(SnIR, 0x0002)// Interrupt
__SOCKET_REGISTER8(SnSR, 0x0003)// Status
__SOCKET_REGISTER16(SnPORT, 0x0004)// Source Port
__SOCKET_REGISTER_N(SnDHAR, 0x0006, 6)// Destination Hardw Addr
__SOCKET_REGISTER_N(SnDIPR, 0x000C, 4)// Destination IP Addr
__SOCKET_REGISTER16(SnDPORT, 0x0010)// Destination Port
__SOCKET_REGISTER16(SnMSSR, 0x0012)// Max Segment Size
__SOCKET_REGISTER8(SnPROTO, 0x0014)// Protocol in IP RAW Mode
__SOCKET_REGISTER8(SnTOS, 0x0015)// IP TOS
__SOCKET_REGISTER8(SnTTL, 0x0016)// IP TTL
__SOCKET_REGISTER16(SnTX_FSR, 0x0020)// TX Free Size
__SOCKET_REGISTER16(SnTX_RD, 0x0022)// TX Read Pointer
__SOCKET_REGISTER16(SnTX_WR, 0x0024)// TX Write Pointer
__SOCKET_REGISTER16(SnRX_RSR, 0x0026)// RX Free Size
__SOCKET_REGISTER16(SnRX_RD, 0x0028)// RX Read Pointer
__SOCKET_REGISTER16(SnRX_WR, 0x002A)// RX Write Pointer (supported?)

#undef __SOCKET_REGISTER8
#undef __SOCKET_REGISTER16
#undef __SOCKET_REGISTER_N

uint8_t readSn8(SOCKET _s, uint16_t _addr) {
	return W5100_read8(CH_BASE + _s * CH_SIZE + _addr);
}

uint8_t writeSn8(SOCKET _s, uint16_t _addr, uint8_t _data) {
	return W5100_write8(CH_BASE + _s * CH_SIZE + _addr, _data);
}

uint16_t readSn16(SOCKET _s, uint16_t _addr, uint8_t *_buf, uint16_t _len) {
	return W5100_read16(CH_BASE + _s * CH_SIZE + _addr, _buf, _len);
}

uint16_t writeSn16(SOCKET _s, uint16_t _addr, uint8_t *_buf, uint16_t _len) {
	return W5100_write16(CH_BASE + _s * CH_SIZE + _addr, _buf, _len);
}

void W5100_execCmdSn(SOCKET s, SockCMD _cmd);

void W5100_getGatewayIp(uint8_t *_addr);
void W5100_setGatewayIp(uint8_t *_addr);
void W5100_getSubnetMask(uint8_t *_addr);
void W5100_setSubnetMask(uint8_t *_addr);
void W5100_getMACAddress(uint8_t *_addr);
void W5100_setMACAddress(uint8_t *_addr);
void W5100_getIPAddress(uint8_t *_addr);
void W5100_setIPAddress(uint8_t *_addr);
void W5100_setRetransmissionTime(uint16_t _timeout);
void W5100_setRetransmissionCount(uint8_t _retry);

// This function writes a byte to the TX buffer (waits for buffer to clear)
void SPI_WriteByte(uint8_t byte);

// This function returns the value of the RX buffer (waits for valid RX data)
uint8_t SPI_ReadByte(void);

#endif
