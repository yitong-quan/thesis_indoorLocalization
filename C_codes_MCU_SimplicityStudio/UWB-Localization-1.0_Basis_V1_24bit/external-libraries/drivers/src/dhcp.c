// DHCP Library v0.3 - April 25, 2009
// Author: Jordan Terrell - blog.jordanterrell.com
// Ported to EFM: Markus Hausser

#include "w5100.h"

#include <string.h>
#include <stdlib.h>
#include "dhcp.h"
#include "util.h"
#include "RTC.h"

uint32_t *baseID = 0;


int Dhcp_beginWithDHCP(SOCKET sock, uint8_t *mac, unsigned long timeout,
		unsigned long responseTimeout) {
	_dhcpLeaseTime = 0;
	_dhcpT1 = 0;
	_dhcpT2 = 0;
	_lastCheck = 0;
	_timeout = timeout;
	_responseTimeout = responseTimeout;

	// zero out _dhcpMacAddr
	memset(_dhcpMacAddr, 0, 6);
	Dhcp_reset_DHCP_lease();

	memcpy((void*) _dhcpMacAddr, (void*) mac, 6);
	_dhcp_state = STATE_DHCP_START;
	return Dhcp_request_DHCP_lease(sock);
}

void Dhcp_reset_DHCP_lease() {
	// zero out _dhcpSubnetMask, _dhcpGatewayIp, _dhcpLocalIp, _dhcpDhcpServerIp, _dhcpDnsServerIp
	memset(_dhcpLocalIp, 0, 20);
}

//return:0 on error, 1 if request is sent and response is received
int Dhcp_request_DHCP_lease(SOCKET sock) {

	uint8_t messageType = 0;

	// Pick an initial transaction ID
	_dhcpTransactionId = Dhcp_random(1UL, 2000UL);
	_dhcpInitialTransactionId = _dhcpTransactionId;

	EthernetUDP_stop(sock);
	if (EthernetUDP_begin(sock, DHCP_CLIENT_PORT) == 0) {
		// Couldn't get a socket
		return 0;
	}

	Dhcp_presend_DHCP();

	int result = 0;

	unsigned long startTime = millis;

	while (_dhcp_state != STATE_DHCP_LEASED) {
		if (_dhcp_state == STATE_DHCP_START) {
			_dhcpTransactionId++;

			Dhcp_send_DHCP_MESSAGE(sock, DHCP_DISCOVER,
					((millis - startTime) / 1000));
			_dhcp_state = STATE_DHCP_DISCOVER;
		} else if (_dhcp_state == STATE_DHCP_REREQUEST) {
			_dhcpTransactionId++;
			Dhcp_send_DHCP_MESSAGE(sock, DHCP_REQUEST,
					((millis - startTime) / 1000));
			_dhcp_state = STATE_DHCP_REQUEST;
		} else if (_dhcp_state == STATE_DHCP_DISCOVER) {
			uint32_t respId = 0;
			messageType = Dhcp_parseDHCPResponse(sock, _responseTimeout,
					&respId);
			if (messageType == DHCP_OFFER) {
				// We'll use the transaction ID that the offer came with,
				// rather than the one we were up to
				_dhcpTransactionId = respId;
				Dhcp_send_DHCP_MESSAGE(sock, DHCP_REQUEST,
						((millis - startTime) / 1000));
				_dhcp_state = STATE_DHCP_REQUEST;
			}
		} else if (_dhcp_state == STATE_DHCP_REQUEST) {
			uint32_t respId = 0;
			messageType = Dhcp_parseDHCPResponse(sock, _responseTimeout,
					&respId);
			if (messageType == DHCP_ACK) {
				_dhcp_state = STATE_DHCP_LEASED;
				result = 1;
				//use default lease time if we didn't get it
				if (_dhcpLeaseTime == 0) {
					_dhcpLeaseTime = DEFAULT_LEASE;
				}
				//calculate T1 & T2 if we didn't get it
				if (_dhcpT1 == 0) {
					//T1 should be 50% of _dhcpLeaseTime
					_dhcpT1 = _dhcpLeaseTime >> 1;
				}
				if (_dhcpT2 == 0) {
					//T2 should be 87.5% (7/8ths) of _dhcpLeaseTime
					_dhcpT2 = _dhcpT1 << 1;
				}
				_renewInSec = _dhcpT1;
				_rebindInSec = _dhcpT2;
			} else if (messageType == DHCP_NAK)
				_dhcp_state = STATE_DHCP_START;
		}

		if (messageType == 255) {
			messageType = 0;
			_dhcp_state = STATE_DHCP_START;
		}

		if (result != 1 && ((millis - startTime) > _timeout))
			break;
		RTC_delay_ms(100);
	}

	// We're done with the socket now
	EthernetUDP_stop(sock);
	_dhcpTransactionId++;

	return result;
}

void Dhcp_presend_DHCP() {
}

void Dhcp_send_DHCP_MESSAGE(SOCKET sock, uint8_t messageType,
		uint16_t secondsElapsed) {
	uint8_t buffer[32];
	memset(buffer, 0, 32);
	uint8_t dest_addr[] = { 255, 255, 255, 255 }; // Broadcast address

	if (-1 == EthernetUDP_beginPacket(sock, dest_addr, DHCP_SERVER_PORT)) {
		// FIXME Need to return errors
		return;
	}

	buffer[0] = DHCP_BOOTREQUEST;   // op
	buffer[1] = DHCP_HTYPE10MB;     // htype
	buffer[2] = DHCP_HLENETHERNET;  // hlen
	buffer[3] = DHCP_HOPS;          // hops

	// xid
	unsigned long xid = htonl(_dhcpTransactionId);
	memcpy(buffer + 4, &(xid), 4);

	// 8, 9 - seconds elapsed
	buffer[8] = ((secondsElapsed & 0xff00) >> 8);
	buffer[9] = (secondsElapsed & 0x00ff);

	// flags
	unsigned short flags = htons(DHCP_FLAGSBROADCAST);
	memcpy(buffer + 10, &(flags), 2);

	// ciaddr: already zeroed
	// yiaddr: already zeroed
	// siaddr: already zeroed
	// giaddr: already zeroed

	//put data in W5100 transmit buffer
	EthernetUDP_write(sock, buffer, 28);

	memset(buffer, 0, 32); // clear local buffer

	memcpy(buffer, _dhcpMacAddr, 6); // chaddr

	//put data in W5100 transmit buffer
	EthernetUDP_write(sock, buffer, 16);

	memset(buffer, 0, 32); // clear local buffer

	// leave zeroed out for sname && file
	// put in W5100 transmit buffer x 6 (192 bytes)

	for (int i = 0; i < 6; i++) {
		EthernetUDP_write(sock, buffer, 32);
	}

	// OPT - Magic Cookie
	buffer[0] = (uint8_t) ((MAGIC_COOKIE >> 24) & 0xFF);
	buffer[1] = (uint8_t) ((MAGIC_COOKIE >> 16) & 0xFF);
	buffer[2] = (uint8_t) ((MAGIC_COOKIE >> 8) & 0xFF);
	buffer[3] = (uint8_t) (MAGIC_COOKIE & 0xFF);

	// OPT - message type
	buffer[4] = dhcpMessageType;
	buffer[5] = 0x01;
	buffer[6] = messageType; //DHCP_REQUEST;

	// OPT - client identifier
	buffer[7] = dhcpClientIdentifier;
	buffer[8] = 0x07;
	buffer[9] = 0x01;
	memcpy(buffer + 10, _dhcpMacAddr, 6);

	// OPT - host name with mac adress
	/*buffer[16] = hostName;
	 buffer[17] = strlen(HOST_NAME) + 6; // length of hostname + last 3 bytes of mac address
	 strcpy((char*) &(buffer[18]), HOST_NAME);

	 Dhcp_printByte((char*) &(buffer[24]), _dhcpMacAddr[3]);
	 Dhcp_printByte((char*) &(buffer[26]), _dhcpMacAddr[4]);
	 Dhcp_printByte((char*) &(buffer[28]), _dhcpMacAddr[5]);*/

	// OPT - host name
	buffer[16] = hostName;
	buffer[17] = strlen(HOST_NAME); // length of hostname
	strcpy((char*) &(buffer[18]), HOST_NAME);
	//Dhcp_printByte((char*) &(buffer[29]), (uint8_t) (*baseID >> 16)); //ID high byte
	//Dhcp_printByte((char*) &(buffer[30]), (uint8_t) (*baseID >> 8)); // ID middle byte
	//Dhcp_printByte((char*) &(buffer[31]), (uint8_t) (*baseID & 0xFF)); // ID low byte

	//put data in W5100 transmit buffer
	EthernetUDP_write(sock, buffer, 30);

	if (messageType == DHCP_REQUEST) {
		buffer[0] = dhcpRequestedIPaddr;
		buffer[1] = 0x04;
		buffer[2] = _dhcpLocalIp[0];
		buffer[3] = _dhcpLocalIp[1];
		buffer[4] = _dhcpLocalIp[2];
		buffer[5] = _dhcpLocalIp[3];

		buffer[6] = dhcpServerIdentifier;
		buffer[7] = 0x04;
		buffer[8] = _dhcpDhcpServerIp[0];
		buffer[9] = _dhcpDhcpServerIp[1];
		buffer[10] = _dhcpDhcpServerIp[2];
		buffer[11] = _dhcpDhcpServerIp[3];

		//put data in W5100 transmit buffer
		EthernetUDP_write(sock, buffer, 12);
	}

	buffer[0] = dhcpParamRequest;
	buffer[1] = 0x06;
	buffer[2] = subnetMask;
	buffer[3] = routersOnSubnet;
	buffer[4] = dns;
	buffer[5] = domainName;
	buffer[6] = dhcpT1value;
	buffer[7] = dhcpT2value;
	buffer[8] = endOption;

	//put data in W5100 transmit buffer
	EthernetUDP_write(sock, buffer, 9);

	EthernetUDP_endPacket(sock);
}

uint8_t Dhcp_parseDHCPResponse(SOCKET sock, unsigned long responseTimeout,
		uint32_t* transactionId) {
	uint8_t type = 0;
	uint8_t opt_len = 0;

	unsigned long startTime = millis;

	while (EthernetUDP_parsePacket(sock) <= 0) {
		if ((millis - startTime) > responseTimeout) {
			return 255;
		}
		SysTick_Delay(50);
	}
	// start reading in the packet
	RIP_MSG_FIXED fixedMsg;
	EthernetUDP_read(sock, (uint8_t*) &fixedMsg, sizeof(RIP_MSG_FIXED));

	if (fixedMsg.op == DHCP_BOOTREPLY
			&& EthernetUDP_remotePort() == DHCP_SERVER_PORT) {
		*transactionId = ntohl(fixedMsg.xid);
		if (memcmp(fixedMsg.chaddr, _dhcpMacAddr, 6) != 0
				|| (*transactionId < _dhcpInitialTransactionId)
				|| (*transactionId > _dhcpTransactionId)) {
			// Need to read the rest of the packet here regardless
			EthernetUDP_flush(sock);
			return 0;
		}

		memcpy(_dhcpLocalIp, fixedMsg.yiaddr, 4);

		// Skip to the option part
		// Doing this a byte at a time so we don't have to put a big buffer
		// on the stack (as we don't have lots of memory lying around)
		for (int i = 0; i < (240 - (int) sizeof(RIP_MSG_FIXED)); i++) {
			EthernetUDP_read_byte(sock); // we don't care about the returned byte
		}

		while (EthernetUDP_available() > 0) {
			switch (EthernetUDP_read_byte(sock)) {
			case endOption:
				break;

			case padOption:
				break;

			case dhcpMessageType:
				opt_len = EthernetUDP_read_byte(sock);
				type = EthernetUDP_read_byte(sock);
				break;

			case subnetMask:
				opt_len = EthernetUDP_read_byte(sock);
				EthernetUDP_read(sock, _dhcpSubnetMask, 4);
				break;

			case routersOnSubnet:
				opt_len = EthernetUDP_read_byte(sock);
				EthernetUDP_read(sock, _dhcpGatewayIp, 4);
				for (int i = 0; i < opt_len - 4; i++) {
					EthernetUDP_read_byte(sock);
				}
				break;

			case dns:
				opt_len = EthernetUDP_read_byte(sock);
				EthernetUDP_read(sock, _dhcpDnsServerIp, 4);
				for (int i = 0; i < opt_len - 4; i++) {
					EthernetUDP_read_byte(sock);
				}
				break;

			case dhcpServerIdentifier:
				opt_len = EthernetUDP_read_byte(sock);
				uint8_t remoteIP[4] = { 0 };
				EthernetUDP_remoteIP(remoteIP);
				if ((_dhcpDhcpServerIp[0] == 0 && _dhcpDhcpServerIp[1] == 0
						&& _dhcpDhcpServerIp[2] == 0
						&& _dhcpDhcpServerIp[3] == 0)
						|| memcmp(_dhcpDhcpServerIp, remoteIP,
								sizeof(_dhcpDhcpServerIp)) == 0) {
					EthernetUDP_read(sock, _dhcpDhcpServerIp,
							sizeof(_dhcpDhcpServerIp));
				} else {
					// Skip over the rest of this option
					while (opt_len--) {
						EthernetUDP_read_byte(sock);
					}
				}
				break;

			case dhcpT1value:
				opt_len = EthernetUDP_read_byte(sock);
				EthernetUDP_read(sock, (uint8_t*) &_dhcpT1, sizeof(_dhcpT1));
				_dhcpT1 = ntohl(_dhcpT1);
				break;

			case dhcpT2value:
				opt_len = EthernetUDP_read_byte(sock);
				EthernetUDP_read(sock, (uint8_t*) &_dhcpT2, sizeof(_dhcpT2));
				_dhcpT2 = ntohl(_dhcpT2);
				break;

			case dhcpIPaddrLeaseTime:
				opt_len = EthernetUDP_read_byte(sock);
				EthernetUDP_read(sock, (uint8_t*) &_dhcpLeaseTime,
						sizeof(_dhcpLeaseTime));
				_dhcpLeaseTime = ntohl(_dhcpLeaseTime);
				_renewInSec = _dhcpLeaseTime;
				break;

			default:
				opt_len = EthernetUDP_read_byte(sock);
				// Skip over the rest of this option
				while (opt_len--) {
					EthernetUDP_read_byte(sock);
				}
				break;
			}
		}
	}

	// Need to skip to end of the packet regardless here
	EthernetUDP_flush(sock);

	return type;
}

/*
 returns:
 0/DHCP_CHECK_NONE: nothing happened
 1/DHCP_CHECK_RENEW_FAIL: renew failed
 2/DHCP_CHECK_RENEW_OK: renew success
 3/DHCP_CHECK_REBIND_FAIL: rebind fail
 4/DHCP_CHECK_REBIND_OK: rebind success
 */
int Dhcp_checkLease(SOCKET sock) {
	//this uses a signed / unsigned trick to deal with millis overflow
	unsigned long now = millis;
	signed long snow = (long) now;
	int rc = DHCP_CHECK_NONE;
	if (_lastCheck != 0) {
		signed long factor;
		//calc how many ms past the timeout we are
		factor = snow - (long) _secTimeout;
		//if on or passed the timeout, reduce the counters
		if (factor >= 0) {
			//next timeout should be now plus 1000 ms minus parts of second in factor
			_secTimeout = snow + 1000 - factor % 1000;
			//how many seconds late are we, minimum 1
			factor = factor / 1000 + 1;

			//reduce the counters by that mouch
			//if we can assume that the cycle time (factor) is fairly constant
			//and if the remainder is less than cycle time * 2
			//do it early instead of late
			if (_renewInSec < factor * 2)
				_renewInSec = 0;
			else
				_renewInSec -= factor;

			if (_rebindInSec < factor * 2)
				_rebindInSec = 0;
			else
				_rebindInSec -= factor;
		}

		//if we have a lease but should renew, do it
		if (_dhcp_state == STATE_DHCP_LEASED && _renewInSec <= 0) {
			_dhcp_state = STATE_DHCP_REREQUEST;
			rc = 1 + Dhcp_request_DHCP_lease(sock);
		}

		//if we have a lease or is renewing but should bind, do it
		if ((_dhcp_state == STATE_DHCP_LEASED || _dhcp_state == STATE_DHCP_START)
				&& _rebindInSec <= 0) {
			//this should basically restart completely
			_dhcp_state = STATE_DHCP_START;
			Dhcp_reset_DHCP_lease();
			rc = 3 + Dhcp_request_DHCP_lease(sock);
		}
	} else {
		_secTimeout = snow + 1000;
	}

	_lastCheck = now;
	return rc;
}

void Dhcp_getLocalIp(uint8_t *ip) {
	ip[0] = _dhcpLocalIp[0];
	ip[1] = _dhcpLocalIp[1];
	ip[2] = _dhcpLocalIp[2];
	ip[3] = _dhcpLocalIp[3];
}

void Dhcp_getSubnetMask(uint8_t *subnet) {
	subnet[0] = _dhcpSubnetMask[0];
	subnet[1] = _dhcpSubnetMask[1];
	subnet[2] = _dhcpSubnetMask[2];
	subnet[3] = _dhcpSubnetMask[3];
}

void Dhcp_getGatewayIp(uint8_t *gatewayIp) {
	gatewayIp[0] = _dhcpGatewayIp[0];
	gatewayIp[1] = _dhcpGatewayIp[1];
	gatewayIp[2] = _dhcpGatewayIp[2];
	gatewayIp[3] = _dhcpGatewayIp[3];
}

void Dhcp_getDhcpServerIp(uint8_t *dhcpIp) {
	dhcpIp[0] = _dhcpDhcpServerIp[0];
	dhcpIp[1] = _dhcpDhcpServerIp[1];
	dhcpIp[2] = _dhcpDhcpServerIp[2];
	dhcpIp[3] = _dhcpDhcpServerIp[3];
}

void Dhcp_getDnsServerIp(uint8_t *dnsIp) {
	dnsIp[0] = _dhcpDnsServerIp[0];
	dnsIp[1] = _dhcpDnsServerIp[1];
	dnsIp[2] = _dhcpDnsServerIp[2];
	dnsIp[3] = _dhcpDnsServerIp[3];
}

void Dhcp_printByte(char * buf, uint8_t n) {
	char *str = &buf[1];
	buf[0] = '0';
	do {
		unsigned long m = n;
		n /= 16;
		char c = m - 16 * n;
		*str-- = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);
}

long Dhcp_random(long howsmall, long howbig) {
	if (howsmall >= howbig) {
		return howsmall;
	}
	long diff = howbig - howsmall;
	return (rand() % diff) + howsmall;
}

void Dhcp_setID(uint32_t *id) {
	baseID = id;
}
