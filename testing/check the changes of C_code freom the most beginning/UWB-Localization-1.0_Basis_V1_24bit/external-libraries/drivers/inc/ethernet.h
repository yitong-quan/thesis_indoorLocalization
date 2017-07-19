#ifndef	_ETHERNET_H_
#define	_ETHERNET_H_

#include "socket.h"
#include "MISC.h"

void Ethernet_init(uint8_t *mac_address, uint8_t *local_ip, uint8_t *gateway, uint8_t *subnet);
int Ethernet_dhcp_init(uint8_t *mac_address, uint8_t *local_ip, uint8_t *gateway, uint8_t *subnet);
void Ethernet_start(SOCKET sock, uint16_t port);
int Ethernet_data_available(SOCKET sock);
int Ethernet_read(SOCKET sock, uint8_t *buf, size_t size);
uint16_t Ethernet_write(SOCKET sock, const uint8_t *buffer, uint16_t size);
uint8_t Ethernet_status(SOCKET sock);
void Ethernet_stop(SOCKET sock);
void Ethernet_interrupt_handler(SOCKET sock);


#endif //_ETHERNET_H_
