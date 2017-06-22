#include "ethernet.h"
#include "dhcp.h"

void Ethernet_init(uint8_t *mac_address, uint8_t *local_ip, uint8_t *gateway, uint8_t *subnet) {
	W5100_init();
	W5100_setMACAddress(mac_address);
	W5100_setGatewayIp(gateway);
	W5100_setSubnetMask(subnet);
	W5100_setIPAddress(local_ip);
}

int Ethernet_dhcp_init(uint8_t *mac_address, uint8_t *local_ip, uint8_t *gateway, uint8_t *subnet) {

	W5100_init();

	/*
	 // SET Gateway
	 W5100_write16(0x0001, gateway, 4);
	 // SET SubnetMask
	 W5100_write16(0x0005, subnet, 4);
	 // SET MAC
	 W5100_write16(0x0009, mac_address, 6);
	 // SET IPAddress
	 W5100_write16(0x000F, local_ip, 4);*/

	W5100_setMACAddress(mac_address);
	uint8_t initIp[] = {0, 0, 0, 0};
	W5100_setIPAddress(initIp);


	// Now try to get our config info from a DHCP server
	int ret = Dhcp_beginWithDHCP(2, mac_address, 5000, 2000);
	if (ret == 1) {
		// We've successfully found a DHCP server and got our configuration info, so set things
		// accordingly
		Dhcp_getLocalIp(local_ip);
		Dhcp_getGatewayIp(gateway);
		Dhcp_getSubnetMask(subnet);
	}

		W5100_setGatewayIp(gateway);
		W5100_setSubnetMask(subnet);
		W5100_setIPAddress(local_ip);


	return ret;

}



void Ethernet_start(SOCKET sock, uint16_t port) {
	if (Ethernet_status(sock) == SnSR_CLOSED) {
		socket(sock, SnMR_TCP, port, 0);
		listen(sock);
	}
}

void Ethernet_stop(SOCKET sock) {
	// attempt to close the connection gracefully (send a FIN to other side)
	disconnect(sock);
	unsigned long start = millis;

	// wait up to a second for the connection to close
	uint8_t s;
	do {
		s = Ethernet_status(sock);
		if (s == SnSR_CLOSED)
			break; // exit the loop
		SysTick_Delay(1);
	} while (millis - start < 1000);

	// if it hasn't closed, close it forcefully
	if (s != SnSR_CLOSED)
		close(sock);
}

int Ethernet_data_available(SOCKET sock) {
	return recvAvailable(sock);
}

int Ethernet_read(SOCKET sock, uint8_t *buf, size_t size) {
	return recv(sock, buf, size);
}

uint16_t Ethernet_write(SOCKET sock, const uint8_t *buffer, uint16_t size) {
	uint16_t n = 0;

	if (Ethernet_status(sock) == SnSR_ESTABLISHED) {
		n += send(sock, buffer, size);
	}

	return n;
}

uint8_t Ethernet_status(SOCKET sock) {
	return socketStatus(sock);
}

void Ethernet_interrupt_handler(SOCKET sock) {
	// Received Fin
	if (W5100_readSnIR(sock) & SnIR_DISCON) {
		Ethernet_stop(sock);
	}
	// Check if connection Timeout
	if (W5100_readSnIR(sock) & SnIR_TIMEOUT) {
		close(sock);
	}
}
